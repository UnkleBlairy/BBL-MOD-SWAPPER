#!/usr/bin/env python3
"""Build ports/index.html: the ports page, in the guide's look (its head and logo script are reused).
Run from anywhere: python3 tools/make_ports_page.py. Edit PORTS below to add or update a port."""
import html, os, re
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import base64
FORM_B64 = "YmxhaXJlbHlvbnNAZ21haWwuY29t"   # put together in the browser: not readable in the page source
PAGE = "https://unkleblairy.github.io/BBL-MOD-SWAPPER/ports/"

# stem, name, version, by, by link, what it is, status (UNCONFIRMED until someone other than its porter tests it)
PORTS = [
    ("JAMBOX", "BBL Jam Box", "0.9", "Blairy (blairysbeatlab)", "https://github.com/UnkleBlairy/BBL-JAM-BOX",
     "Live-jamming groovebox: 6 tracks, p-locks, trig conditions, the Sample Lab.", "UNCONFIRMED"),
    ("JIANT", "JIANT FM", "0.9.18", "juanjiant-bit", "https://github.com/juanjiant-bit/JIANT",
     "Six tracks, eleven synth engines, a 64-step sequencer, songs and a generator.", "UNCONFIRMED"),
    ("SLOOP", "SLOOP", "2.8.5", "isod89", "https://github.com/isod89/sloop-fm1",
     "Four-track groovebox: three synths and a drum machine, 37 kits, your own samples through the Sample Lab (FX + GLO).", "UNCONFIRMED"),
    ("X0X", "X0X", "1.2.2", "Charles Vestal", "https://github.com/charlesvestal/fm1-x0x",
     "A 909, an 808, two 303s with an acid generator and a breakbeat player, all at once.", "UNCONFIRMED"),
]

g = open(os.path.join(ROOT, "guide/index.html"), encoding="utf-8").read()
head = g[:g.index("</style>")].replace("<title>BBL Mod Swapper build guide</title>", "<title>BBL Mod Swapper ports</title>")
logo = g[g.index("<script>", g.index("<footer")):g.index("</script>", g.index("<footer")) + 9]
prompt = open(os.path.join(ROOT, "ports/PORTING_PROMPT.md"), encoding="utf-8").read()
prompt = re.search(r"```text\n(.*?)```", prompt, re.S).group(1).strip()

def mb(stem):
    return os.path.getsize(os.path.join(ROOT, f"ports/{stem}/{stem}.zip")) / 1e6

css = """
.btn { display: inline-block; background: var(--pink); color: #000; text-decoration: none; padding: 6px 14px; border: 0;
  font: inherit; cursor: pointer; }
.btn:hover, .btn:focus-visible { background: #000; color: #fff; outline: none; }
.btn.big { font-size: 18px; padding: 10px 18px; }
.port { border-top: 2px solid #000; padding: 12px 0 4px; }
.port:first-of-type { border-top: 0; padding-top: 0; }
.port .row { display: flex; flex-wrap: wrap; align-items: baseline; gap: 4px 12px; margin-bottom: 4px; }
.port .name { font-size: 20px; }
.port .ver { color: #555; }
.chip { font-size: 13px; padding: 1px 8px; border: 2px solid #c8006a; color: #c8006a; }
.chip.ok { border-color: #000; color: #000; }
.port .by { font-size: 14px; color: #444; margin-bottom: 6px; }
.port p { margin-bottom: 8px; }
.size { font-size: 13px; color: #555; margin-left: 8px; }
form { display: grid; gap: 12px; }
label { display: grid; gap: 4px; font-size: 14px; }
label span { color: #c8006a; }
input, select, textarea { font: 15px/1.4 "BLBPixel", ui-monospace, monospace; letter-spacing: .03em; color: #000;
  background: #fff; border: 2px solid #000; padding: 6px 8px; border-radius: 0; width: 100%; box-sizing: border-box; }
input:focus, select:focus, textarea:focus { outline: 3px solid var(--pink); outline-offset: 0; }
textarea { min-height: 120px; resize: vertical; }
.two { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
@media (max-width: 520px) { .two { grid-template-columns: 1fr; } }
.hp { position: absolute; left: -9999px; }
.sent { background: var(--pink); color: #000; padding: 10px 14px; margin: 18px 0 0; display: none; }
.btn.sm { padding: 2px 10px; font-size: 14px; }
a.src { font-size: 13px; color: #555; }
#more td { vertical-align: middle; }
#more .ver { color: #777; font-size: 13px; }
#find { margin-bottom: 10px; width: calc(100% - 32px); }
pre.prompt { background: #000; color: #fff; padding: 12px; font: 13px/1.45 ui-monospace, monospace; white-space: pre-wrap;
  max-height: 320px; overflow: auto; margin-bottom: 10px; }
"""

cards = []
for stem, name, ver, by, link, what, status in PORTS:
    cls = "chip ok" if status == "CONFIRMED" else "chip"
    cards.append(f"""  <div class="port">
    <div class="row"><span class="name">{html.escape(name)}</span><span class="ver">{ver}</span><span class="{cls}">{status}</span></div>
    <div class="by">By <a href="{link}" target="_blank" rel="noopener">{html.escape(by)}</a>. Ported by Blairy.</div>
    <p>{html.escape(what)}</p>
    <a class="btn" href="{stem}/{stem}.zip" download>DOWNLOAD</a><span class="size">{mb(stem):.1f} MB zip</span>
  </div>""")
MORE = [l.rstrip("\n").split("\t") for l in open(os.path.join(ROOT, "tools/more_ports.tsv"), encoding="utf-8") if l.strip()]
MORE.sort(key=lambda r: r[1].lower())
rows = []
for stem, name, ver, owner, url, lic in MORE:
    rows.append(f"""    <tr data-n="{html.escape((name + ' ' + owner).lower())}"><td>{html.escape(name)} <span class="ver">{html.escape(ver)}</span></td>
      <td><a href="{url}" target="_blank" rel="noopener">{html.escape(owner)}</a></td><td>{lic}</td>
      <td><a class="btn sm" href="{stem}/{stem}.zip" download>DOWNLOAD</a> <a class="src" href="{stem}/{stem}-source.zip" download>source</a></td></tr>""")
total = os.path.getsize(os.path.join(ROOT, "ports/ALL-PORTS.zip")) / 1e6
opts = "\n".join([f'          <option>{html.escape(n)} {v}</option>' for _, n, v, *_ in PORTS] + [f'          <option>{html.escape(n)} {v} (untested)</option>' for _, n, v, *_ in MORE])

body = f"""{css}</style>
</head>
<body>
<main>
<header class="top">
  <canvas id="logo" width="240" height="214" aria-label="BBL MOD SWAPPER"></canvas>
  <div><h1>PORTS</h1><p>Firmwares for your Mod Swapper. Download, copy to the SD card, pick them in the menu.</p></div>
</header>

<nav class="toc">
  <a href="#list">LIST</a><a href="#more">MORE PORTS ({len(MORE)})</a><a href="#card">ONTO THE CARD</a><a href="#report">REPORT</a><a href="#port">PORT YOUR OWN</a><a href="../guide/">BUILD GUIDE</a>
</nav>

<div class="sent" id="sent" role="status">Thanks! Your report is on its way to Blairy.</div>

<section class="pop" id="list">
  <h2>Ports</h2>
  <p><a class="btn big" href="ALL-PORTS.zip" download>DOWNLOAD ALL</a><span class="size">{total:.1f} MB zip, every port</span></p>
  <p class="note">Felucca (by Leo Kuroshita) is built into the Teensy: it's always in the menu, no download needed.</p>
{chr(10).join(cards)}
  <p class="note"><b class="pink">UNCONFIRMED</b> means only its porter has tested it so far. Tried one? Tell us below, it helps everyone.</p>
</section>

<section class="pop" id="more">
  <h2>More ports: {len(MORE)} untested</h2>
  <p>FM-1 firmwares by other people, ported to the Mod Swapper in one big batch. <b class="warn">Not tried on hardware yet.</b> Some will work, some won't: that's what the reports are for.</p>
  <p class="note">The menu shows every firmware on the card (up to 160) from release 1.1, so load as many as you like. Tried one? <a href="#report">Tell us</a> if it works or not, and it moves up to the list above.</p>
  <input id="find" type="search" placeholder="Search {len(MORE)} ports by name or author..." aria-label="Search ports">
  <div class="tbl"><table>
    <tr><th>Port</th><th>Original by</th><th>Licence</th><th></th></tr>
{chr(10).join(rows)}
  </table></div>
  <p class="note">Every one keeps its original authors' credits and licence. "source" is the code this build was made from, with the porting changes.</p>
</section>

<section class="pop" id="card">
  <h2>Onto the SD card</h2>
  <ol>
    <li>Press <b>DOWNLOAD</b> on a port, or <b>DOWNLOAD ALL</b>.</li>
    <li>Unzip it. Inside is a folder called <kbd>ROMS</kbd>.</li>
    <li>Copy that <kbd>ROMS</kbd> folder to the top of your SD card. If the card asks, let the files go in with the ones already there.</li>
    <li>Card back in the Teensy, switch the FM-1 on, pick the port in the menu.</li>
  </ol>
  <p class="note">Each port is three files, <kbd>NAME.R1</kbd> <kbd>NAME.R2</kbd> <kbd>NAME.R3</kbd>: keep all three. To remove a port, delete its three files from <kbd>ROMS</kbd>. The menu shows up to 160 firmwares (release 1.1).</p>
</section>

<section class="pop" id="report">
  <h2>Report</h2>
  <p>A bug, a port that works, a port you made, a question: it goes straight to Blairy's inbox.</p>
  <form id="report-form" action="#" method="POST" enctype="multipart/form-data">
    <input type="hidden" name="_subject" value="Mod Swapper report">
    <input type="hidden" name="_template" value="table">
    <input type="hidden" name="_next" value="{PAGE}?sent=1#sent">
    <input class="hp" type="text" name="_honey" tabindex="-1" autocomplete="off" aria-hidden="true">
    <div class="two">
      <label><span>What is it</span>
        <select name="Type" required>
          <option>Bug</option><option>It works</option><option>New port</option><option>Question</option><option>Other</option>
        </select></label>
      <label><span>Which port</span>
        <select name="Port">
          <option>Not about one port</option>
          <option>Felucca (built in)</option>
{opts}
          <option>My own port</option>
        </select></label>
    </div>
    <div class="two">
      <label><span>Your name</span><input name="Name" required autocomplete="name"></label>
      <label><span>Your email (so Blairy can reply)</span><input type="email" name="email" required autocomplete="email"></label>
    </div>
    <div class="two">
      <label><span>Teensy PSRAM</span>
        <select name="PSRAM"><option>Not sure</option><option>None</option><option>One chip</option><option>Two chips</option></select></label>
      <label><span>Your FM-1 (when bought, anything different)</span><input name="FM-1"></label>
    </div>
    <label><span>What happened</span><textarea name="Message" required placeholder="What you did, what you expected, what you got. For a new port: what it is, the original's link and licence, what you tested."></textarea></label>
    <label><span>Attach a file (optional, up to 5 MB): a photo, or a zip of your port</span><input type="file" name="attachment"></label>
    <p><button class="btn big" type="submit">SEND</button></p>
  </form>
</section>

<section class="pop" id="port">
  <h2>Port your own</h2>
  <p>You don't need to be a coder. An AI coding helper (Claude Code, Codex, Cursor and the like) can do the porting.</p>
  <ol>
    <li>Pick an FM-1 firmware whose licence allows it (GPL, MIT and the like).</li>
    <li>Press <b>COPY PROMPT</b>, paste it into your AI helper, and change the first line to the firmware's name and link.</li>
    <li>It builds the three <kbd>.R1</kbd> <kbd>.R2</kbd> <kbd>.R3</kbd> files. Put them on your card and test them.</li>
    <li>Send it with the <a href="#report">form above</a>: choose <b>New port</b> and attach a zip of the three files and your changed source. It goes on this page with your name on it.</li>
  </ol>
  <p><button class="btn big" type="button" id="copy">COPY PROMPT</button></p>
  <pre class="prompt" id="prompt">{html.escape(prompt)}</pre>
  <p class="note">The source code the prompt uses is in the <a href="https://github.com/UnkleBlairy/BBL-MOD-SWAPPER/tree/gh-pages/source" target="_blank" rel="noopener">source folder</a>.</p>
</section>

<footer>
  BBL Mod Swapper by Blairy (blairysbeatlab). Every port keeps its own authors' credits and licence (GPL-3.0). No warranty.<br>
  Not made or supported by M-VAVE or PJRC. M-VAVE, FM-1 and Teensy are trademarks of their respective owners.
</footer>
</main>
{logo}
<script>
document.getElementById('report-form').action = 'https://formsubmit.co/' + atob('{FORM_B64}');
document.getElementById('find').addEventListener('input', e => {{
  const q = e.target.value.trim().toLowerCase();
  document.querySelectorAll('#more tr[data-n]').forEach(r => r.style.display = !q || r.dataset.n.includes(q) ? '' : 'none');
}});
if (new URLSearchParams(location.search).has('sent')) document.getElementById('sent').style.display = 'block';
document.getElementById('copy').addEventListener('click', async (e) => {{
  const t = document.getElementById('prompt').textContent;
  try {{ await navigator.clipboard.writeText(t); }} catch (_) {{
    const r = document.createRange(); r.selectNodeContents(document.getElementById('prompt'));
    const s = getSelection(); s.removeAllRanges(); s.addRange(r); document.execCommand('copy');
  }}
  e.target.textContent = 'COPIED'; setTimeout(() => e.target.textContent = 'COPY PROMPT', 2000);
}});
</script>
</body>
</html>
"""
open(os.path.join(ROOT, "ports/index.html"), "w", encoding="utf-8").write(head + body)
print("ports/index.html written")
