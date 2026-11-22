import subprocess
import sys
import tempfile
from pathlib import Path

host = Path(sys.argv[1]).resolve()
ground = Path(__file__).resolve().parents[1] / "tools/bench_frame.py"
with tempfile.TemporaryDirectory() as scratch:
    iq = Path(scratch) / "frame.iq"
    for rate in ("1200", "9600"):
        subprocess.run([sys.executable, ground, "encode", "1", "4660", "010203", "--rate", rate,
                        "--if-hz", "12000", "--iq", iq], check=True)
        subprocess.run([host, "read", iq, rate], check=True)
        subprocess.run([host, "write", iq, rate], check=True)
        line = subprocess.check_output([sys.executable, ground, "demod", iq, "--rate", rate], text=True)
        assert line.strip() == "kind=1 sequence=4660 payload=010203", line
