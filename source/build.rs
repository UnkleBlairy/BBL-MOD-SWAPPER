//! teensy-fm1 build: the memory layout, and Felucca (C, GPL-3.0) compiled for the Cortex-M7.
use imxrt_rt::{Family, FlexRamBanks, Memory, RuntimeBuilder};
use std::{env, fs, path::PathBuf, process::Command};

/// Which firmware this build is (TFM1_FW): slot 0 holds Felucca and the firmware menu, slot 1 the Jam Box, slot 2 JIANT.
struct Fw {
    dir: &'static str,
    version: &'static str,
    name: &'static str,
    origin: u32,
    length: u32,
    includes: &'static [&'static str],
    text_in_flash: bool, // code too big for ITCM: run it from flash (through the 32 KB I-cache)
    itcm_banks: u32,     // FlexRAM: 32 KB banks of ITCM (code); the rest of the 16 is DTCM (data, stack)
    extra: &'static [&'static str],   // more C files (X0X: its DSP and sequencer units)
    defines: &'static [&'static str],
    stem: &'static str, // its name on the card: ROMS/<STEM>.R1.., DATA/<STEM>.NOR
    nor: u32,           // its save memory (the FM-1's SPI NOR) in its own slot, below the tag; 0: none (RAM only)
}
const FELUCCA: Fw = Fw {
    dir: "ports/felucca",
    version: "v1.8.1",
    name: "Felucca",
    origin: 0x6000_0000,
    length: 0x20_0000,
    includes: &["build/gen", "firmware/src", "firmware/hal", "firmware/bsp/chip", "firmware/bsp/boards/fm1"],
    text_in_flash: false,
    itcm_banks: 12,
    extra: &[],
    defines: &[],
    stem: "FELUCCA",
    nor: 0,
};
const JAMBOX: Fw = Fw {
    dir: "ports/jambox",
    version: "0.9",
    name: "BBL Jam Box",
    origin: 0x6020_0000,
    length: 0x20_0000,
    includes: &["build/gen", "firmware/src", "firmware/hal"],
    text_in_flash: false,
    itcm_banks: 12,
    extra: &[],
    defines: &[],
    stem: "JAMBOX",
    nor: 0x10_0000,
};
const JIANT: Fw = Fw {
    dir: "ports/jiant",
    version: "v0.9.18",
    name: "JIANT FM",
    origin: 0x6040_0000,
    length: 0x3C_0000,
    includes: &["build/gen", "firmware/src", "firmware/hal"],
    text_in_flash: false, // ~600 KB of code, ITCM is 384 KB: ports/jiant/teensy_flash_funcs.txt stays in flash
    itcm_banks: 12,
    extra: &[],
    defines: &[],
    stem: "JIANT",
    nor: 0x7_0000, // its part of the SPI NOR, 0x90000.. (songs, settings; felucca_teensy.c NOR_LO), mirrored to the card
};

const SLOOP: Fw = Fw {
    dir: "ports/sloop",
    version: "2.8.5",
    name: "SLOOP",
    origin: 0x6060_0000,
    length: 0x1B_F000,
    includes: &["build/gen", "firmware/src", "firmware/hal"],
    text_in_flash: false,
    itcm_banks: 10, // 220 KB of code, 83 KB of data that must be in DTCM
    extra: &[],
    defines: &[],
    stem: "SLOOP",
    nor: 0x10_0000, // its 1 MiB SPI NOR (settings, projects, presets, the user sample slots), mirrored to the card
};

const X0X: Fw = Fw {
    dir: "ports/x0x",
    version: "1.2.2",
    name: "X0X",
    origin: 0x6060_0000,
    length: 0x1B_F000,
    includes: &["build/gen", "firmware/src", "firmware/src/dsp"],
    text_in_flash: false,
    itcm_banks: 10,
    extra: &[
        "firmware/src/dsp/drum909.c", "firmware/src/dsp/drum808.c", "firmware/src/dsp/bass303.c",
        "firmware/src/dsp/breaks.c", "firmware/src/dsp/fxbus.c", "firmware/src/dsp/master.c",
        "firmware/src/seq/sequencer.c", "firmware/src/seq/tb3po.c", "firmware/src/seq/pattern.c",
        "firmware/src/seq/motion.c", "firmware/src/app/engine.c",
    ],
    defines: &["X0X_HOST", "X0X_WEB"],
    stem: "X0X",
    nor: 0,
};

fn main() {
    println!("cargo:rerun-if-env-changed=TFM1_FW");
    println!("cargo:rustc-check-cfg=cfg(fw_menu)");
    let fw = match env::var("TFM1_FW").as_deref() {
        Ok("jambox") => JAMBOX,
        Ok("jiant") => JIANT,
        Ok("sloop") => SLOOP,
        Ok("x0x") => X0X,
        Ok("felucca") | Err(_) => FELUCCA,
        Ok(x) => panic!("TFM1_FW={x}: felucca, jambox, jiant or sloop"),
    };
    // TFM1_SLOT=1..3: build a ROM for that flash slot (roms.rs: slot s at BASE[s], its last 4 KB is the tag)
    println!("cargo:rerun-if-env-changed=TFM1_SLOT");
    let mut fw = fw;
    const BASE: [u32; 4] = [0x6000_0000, 0x6020_0000, 0x6040_0000, 0x6060_0000];
    const END: [u32; 4] = [0x6020_0000, 0x6040_0000, 0x6060_0000, 0x607C_0000];
    let slot: usize = match env::var("TFM1_SLOT") {
        Ok(v) => v.parse().expect("TFM1_SLOT: 1, 2 or 3"),
        Err(_) => BASE.iter().position(|&b| b == fw.origin).expect("origin"),
    };
    assert!(slot < 4 && (slot == 0) == (fw.origin == 0x6000_0000), "slot 0 is Felucca's");
    fw.origin = BASE[slot];
    fw.length = if slot == 0 { END[0] - BASE[0] } else { END[slot] - 0x1000 - BASE[slot] };
    // the save memory: the top of the slot under the tag (the image must end below it). Slot 0 keeps it in RAM.
    let nor_addr = if slot > 0 && fw.nor > 0 { END[slot] - 0x1000 - fw.nor } else { 0 };
    if nor_addr != 0 {
        fw.length = nor_addr - BASE[slot];
    }
    println!("cargo:rustc-env=TFM1_STEM={}", fw.stem);
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(
        out_dir.join("fwcfg.rs"),
        format!("pub const NOR_ADDR: u32 = {nor_addr:#x};\npub const NOR_SIZE: u32 = {:#x};\npub const STEM: &str = \"{}\";\npub const TAG_ADDR: u32 = {:#x};\n", if nor_addr != 0 { fw.nor } else { 0 }, fw.stem, END[slot] - 0x1000),
    )
    .unwrap();
    // the Sample Lab (src/lab.rs) is in the runtime of every mod that has a sample hook (src/hooks/)
    println!("cargo:rustc-check-cfg=cfg(fw_lab)");
    println!("cargo:rustc-check-cfg=cfg(fw_hook_jambox)");
    println!("cargo:rustc-check-cfg=cfg(fw_rec)"); // the Lab's recorder (line in): the Jam Box only, for now
    println!("cargo:rustc-check-cfg=cfg(fw_hook_sloop)");
    if fw.stem == "JAMBOX" && nor_addr != 0 {
        println!("cargo:rustc-cfg=fw_lab");
        println!("cargo:rustc-cfg=fw_hook_jambox");
        println!("cargo:rustc-cfg=fw_rec");
    }
    println!("cargo:rustc-check-cfg=cfg(fw_hook_none)");
    if fw.stem == "SLOOP" && nor_addr != 0 {
        println!("cargo:rustc-cfg=fw_lab");
        println!("cargo:rustc-cfg=fw_hook_sloop");
    }
    // every slot ROM has the overlay (save states, src/states.rs); a mod without a sample hook has no Sample Lab page
    if slot > 0 && !((fw.stem == "JAMBOX" || fw.stem == "SLOOP") && nor_addr != 0) {
        println!("cargo:rustc-cfg=fw_lab");
        println!("cargo:rustc-cfg=fw_hook_none");
    }
    if fw.origin == 0x6000_0000 {
        println!("cargo:rustc-cfg=fw_menu"); // slot 0: the boot menu, and the jump to the other slot
    }
    println!("cargo:rustc-check-cfg=cfg(fw_text_flash)");
    if fw.text_in_flash {
        println!("cargo:rustc-cfg=fw_text_flash"); // ITCM is free: the profiler keeps its samples there
    }
    println!("cargo:rustc-env=TFM1_NAME={}", fw.name);
    println!("cargo:rustc-env=TFM1_VERSION={}", fw.version);
    // FlexRAM: 384 KB ITCM (all code), 128 KB DTCM (data, stack). OCRAM2 (512 KB): big buffers.
    RuntimeBuilder::from_flexspi(Family::Imxrt1060, 7936 * 1024)
        .flexram_banks(FlexRamBanks { ocram: 0, itcm: fw.itcm_banks, dtcm: 16 - fw.itcm_banks })
        .text(if fw.text_in_flash { Memory::Flash } else { Memory::Itcm })
        .rodata(Memory::Flash)
        .vectors(Memory::Dtcm)
        .data(Memory::Dtcm)
        .bss(Memory::Dtcm)
        .uninit(Memory::Ocram)
        .stack(Memory::Dtcm)
        .stack_size(32 * 1024)
        .heap(Memory::Ocram)
        .heap_size(0)
        .linker_script_name("fm1link.x")
        .build()
        .unwrap();

    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", fw.dir);

    // this firmware's slot in the flash: the runtime's script places everything (boot header included) from ORIGIN(FLASH)
    let ld = out.join("fm1link.x");
    let script = fs::read_to_string(&ld).expect("fm1link.x");
    let from = "FLASH (RX) : ORIGIN = 0x60000000, LENGTH = 0x7C0000";
    assert!(script.contains(from), "fm1link.x: the FLASH line moved");
    let script = script.replace(from, &format!("FLASH (RX) : ORIGIN = {:#x}, LENGTH = {:#x}", fw.origin, fw.length));
    // the SD card, its FAT library and the ROM library run from flash: not time critical, and ITCM is full. (Never
    // the flash writer, flashw.rs: it runs while the flash is busy.)
    let xip = "*(.xip .xip.*);";
    assert!(script.contains(xip), "fm1link.x: the .xip rule moved");
    let script = script.replace(
        xip,
        "*(.xip .xip.*);\n    *(.text.*embedded_sdmmc*);\n    *(.text.*teensy_fm14roms*);\n    *(.text.*teensy_fm12sd*);\n    *(.text.*teensy_fm13nor*);\n    *(.text.*teensy_fm13lab*);\n    *(.text.*teensy_fm15hooks*);\n    *(.text.*teensy_fm16states*);\n    *(.text.*usb_device*);\n    *(.text.*usbd_serial*);\n    *(.text.*core3fmt*);",
    );
    // the mod's own memory (the C library's data, bss and OCRAM buffers) first in each section, between marks: a save
    // state (src/states.rs) is a copy of those three ranges. Only the mod writes them, and only from the main loop.
    let mut script = script;
    for (pat, sec) in [("*(.data .data.*);", "data"), ("*(.bss .bss.*);", "bss"), ("*(.uninit .uninit.*);", "uninit")] {
        assert!(script.matches(pat).count() == 1, "fm1link.x: the {sec} rule moved");
        script = script.replace(
            pat,
            &format!(". = ALIGN(4);\n    __smod_{sec} = .;\n    *libfelucca.a:*(.{sec} .{sec}.*);\n    . = ALIGN(4);\n    __emod_{sec} = .;\n    {pat}"),
        );
    }
    fs::write(&ld, script).unwrap();

    // the firmware's browser build (Felucca: hugelton/Felucca, ports/felucca/SOURCE_COMMIT.txt; the Jam Box: ports/jambox)
    let mut cc = cc::Build::new();
    cc.file(format!("{}/web/emu/felucca_teensy.c", fw.dir))
        .define("FELUCCA_VERSION", format!("\"{}\"", fw.version).as_str());
    if nor_addr != 0 {
        cc.define("TEENSY_NOR_ADDR", format!("{nor_addr:#x}u").as_str());
    }
    for i in fw.includes {
        cc.include(format!("{}/{}", fw.dir, i));
    }
    for f in fw.extra {
        cc.file(format!("{}/{}", fw.dir, f));
    }
    for d in fw.defines {
        cc.define(d, None);
    }
    let objects = cc
        .flag("-mcpu=cortex-m7")
        .flag("-mthumb")
        .flag("-mfloat-abi=hard")
        .flag("-mfpu=fpv5-d16")
        .flag("-std=gnu99")
        .flag("-ffp-contract=off")
        .flag("-ffunction-sections")
        .flag("-fdata-sections")
        .flag("-w")
        .opt_level(2)
        .cargo_metadata(false)
        .compile_intermediates();

    // Felucca's largest zero-initialised buffers, biggest first while they fit OCRAM: renamed into the runtime's
    // .uninit section (OCRAM, not zeroed by the runtime: main.rs zeroes __suninit..__euninit before Felucca starts).
    // (the Jam Box fork: a little less: its track recording's state: ports/jambox/docs/TRACK_REC.md)
    let ocram_budget: u64 = if fw.stem == "JAMBOX" { 494 } else { 508 } * 1024;
    let mut bss: Vec<(String, u64)> = Vec::new();
    for o in &objects {
        let out = Command::new("arm-none-eabi-size").arg("-A").arg(o).output().expect("arm-none-eabi-size");
        for l in String::from_utf8_lossy(&out.stdout).lines() {
            let f: Vec<&str> = l.split_whitespace().collect();
            if f.len() >= 2 && f[0].starts_with(".bss.") {
                if let Ok(n) = f[1].parse::<u64>() {
                    bss.push((f[0][5..].to_string(), n));
                }
            }
        }
    }
    bss.sort_by(|a, b| b.1.cmp(&a.1));
    let mut used = 0u64;
    let mut big: Vec<String> = Vec::new();
    for (name, n) in &bss {
        let n = (n + 7) & !7;
        if n >= 1024 && used + n <= ocram_budget {
            used += n;
            big.push(name.clone());
        }
    }
    println!("cargo:warning=OCRAM: {} buffers, {} KB", big.len(), used / 1024);
    let mut objs = Vec::new();
    for o in objects {
        let mut cmd = Command::new("arm-none-eabi-objcopy");
        for b in &big {
            cmd.arg(format!("--rename-section=.bss.{b}=.uninit.{b}"));
        }
        // functions kept in flash (the runtime's .xip section) when the code is too big for ITCM
        if let Ok(list) = fs::read_to_string(format!("{}/teensy_flash_funcs.txt", fw.dir)) {
            for f in list.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')) {
                cmd.arg(format!("--rename-section=.text.{f}=.xip.{f}"));
            }
        }
        let renamed = out.join(format!("ocram-{}", o.file_name().unwrap().to_string_lossy()));
        let st = cmd.arg(&o).arg(&renamed).status().expect("arm-none-eabi-objcopy");
        assert!(st.success(), "objcopy failed");
        objs.push(renamed);
    }
    let lib = out.join("libfelucca.a");
    let _ = fs::remove_file(&lib);
    let st = Command::new("arm-none-eabi-ar").arg("crs").arg(&lib).args(&objs).status().expect("arm-none-eabi-ar");
    assert!(st.success(), "ar failed");
    println!("cargo:rustc-link-search=native={}", out.display());
    println!("cargo:rustc-link-lib=static=felucca");
}
