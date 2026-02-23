import numpy as np
import pandas as pd
from io import StringIO
from pathlib import Path
import re

FILE = 'FREQ_SWEEP_1.csv'
DIR = '/Users/jacobdebry/Documents/HallArray/Pico/Scripts/Plot Data/Frequency Sweep'
FILE_PATH = f'{DIR}/{FILE}'
OUT_FOLDER = f'{DIR}/Split'

BEGIN_RE = re.compile(r"^===\s*BEGIN\s*(.*?)\s*===\s*$")
END_RE = re.compile(r"^===\s*END\s*===")

def split_file(PATH):
    sections = []
    current_name = None
    current_lines = []

    with open(PATH, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip("\n")

            m = BEGIN_RE.match(line)
            if m:
                print("BEGIN", m.group(1))
                current_name = m.group(1).strip() or f"sweep_{len(sections)+1}"
                current_lines = []
                continue

            if END_RE.match(line):
                if current_name is not None and current_lines:
                    block_text = "\n".join(current_lines)
                    df = pd.read_csv(StringIO(block_text), header=None)
                    sections.append((current_name, df))
                current_name = None
                current_lines = []
                continue

            if current_name is not None:
                current_lines.append(line)
    # Handle a trailing section if the file ends without an explicit END marker.
    if current_name is not None and current_lines:
        block_text = "\n".join(current_lines)
        df = pd.read_csv(StringIO(block_text), header=None)
        sections.append((current_name, df))

    return sections

sections = split_file(FILE_PATH)
out_dir = Path(OUT_FOLDER)
out_dir.mkdir(parents=True, exist_ok=True)

for i, (name, df) in enumerate(sections, start=1):
    safe = re.sub(r"[^A-Za-z0-9._-]+", "_", name) or f"sweep_{i}"
    out_file = out_dir / f"{i:03d}_{safe}.csv"
    df.to_csv(out_file, index=False, header=False)
    print(f"Saved: {out_file} ({len(df)} rows)")
