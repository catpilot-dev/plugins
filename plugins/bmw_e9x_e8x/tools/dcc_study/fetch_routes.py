"""Pull rlogs of engaged routes from the C3 via the Connect-on-Device API."""
import argparse
import subprocess
import tarfile
import tempfile
import urllib.parse
from pathlib import Path

import requests

from common import ROUTES_DIR


def c3_host():
  out = subprocess.run(["ssh", "-G", "c3"], capture_output=True, text=True,
                       check=True).stdout
  for line in out.splitlines():
    if line.startswith("hostname "):
      return line.split()[1]
  raise RuntimeError("could not resolve host from `ssh -G c3`")


def route_url_name(fullname):
  return urllib.parse.quote(fullname.replace("/", "|"), safe="")


def extract_rlogs(tar_path, dest_dir):
  n = 0
  with tarfile.open(tar_path, "r:gz") as tf:
    for m in tf.getmembers():
      p = Path(m.name)
      if m.isfile() and p.name == "rlog.zst":
        seg_dir = Path(dest_dir) / p.parent.name
        seg_dir.mkdir(parents=True, exist_ok=True)
        (seg_dir / "rlog.zst").write_bytes(tf.extractfile(m).read())
        n += 1
  return n


def main():
  p = argparse.ArgumentParser()
  p.add_argument("--host", default=None, help="override `ssh -G c3` resolution")
  p.add_argument("--limit", type=int, default=100)
  p.add_argument("--min-engagement", type=int, default=1)
  p.add_argument("--route", action="append", default=None,
                 help="route date string, repeatable; bypasses engagement filter")
  args = p.parse_args()

  host = args.host or c3_host()
  base = f"http://{host}:8082"
  try:
    devices = requests.get(f"{base}/v1/me/devices/", timeout=10).json()
  except requests.ConnectionError as e:
    raise SystemExit(f"COD unreachable at {base} — is the C3 on and awake? "
                     f"(check `ssh c3`)\n{e}")
  dongle = devices[0]["dongle_id"]
  routes = requests.get(f"{base}/v1/devices/{dongle}/routes",
                        params={"limit": args.limit}, timeout=30).json()

  ROUTES_DIR.mkdir(parents=True, exist_ok=True)
  for r in routes:
    date = r["fullname"].split("/")[1]
    if args.route is not None and date not in args.route:
      continue
    name = route_url_name(r["fullname"])
    meta = requests.get(f"{base}/v1/route/{name}/", timeout=120).json()
    pct = meta.get("engagement_pct") or 0
    if args.route is None and pct < args.min_engagement:
      print(f"skip {date}: engagement {pct}%")
      continue
    n_seg = (meta.get("maxqlog") or 0) + 1
    if all((ROUTES_DIR / f"{date}--{s}" / "rlog.zst").exists() for s in range(n_seg)):
      print(f"have {date} ({n_seg} segments)")
      continue
    print(f"downloading {date} (engagement {pct}%, {n_seg} segments)...")
    with requests.get(f"{base}/v1/route/{name}/download",
                      params={"files": "rlog"}, stream=True, timeout=600) as resp:
      resp.raise_for_status()
      with tempfile.NamedTemporaryFile(suffix=".tar.gz") as tmp:
        for chunk in resp.iter_content(1 << 20):
          tmp.write(chunk)
        tmp.flush()
        print(f"  {extract_rlogs(tmp.name, ROUTES_DIR)} rlogs extracted")


if __name__ == "__main__":
  main()
