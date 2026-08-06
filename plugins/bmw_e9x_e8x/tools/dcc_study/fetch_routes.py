"""Pull rlogs of engaged routes from the C3 via the Connect-on-Device API."""
import argparse
import subprocess
import urllib.parse
from pathlib import Path

import requests

from common import ROUTES_DIR

# COD's server.py defaults to port 80 on the device (API.md's 8082 is a dev-server
# convention). Override with --port if the device runs it elsewhere.
DEFAULT_PORT = 80


def c3_host():
  out = subprocess.run(["ssh", "-G", "c3"], capture_output=True, text=True,
                       check=True).stdout
  for line in out.splitlines():
    if line.startswith("hostname "):
      return line.split()[1]
  raise RuntimeError("could not resolve host from `ssh -G c3`")


def route_url_name(fullname):
  return urllib.parse.quote(fullname.replace("/", "|"), safe="")


def download_segments(log_urls, date, dest_dir, session=None):
  """Fetch each segment's rlog.zst into <dest_dir>/<date>--<seg>/rlog.zst.

  Uses the per-segment /connectdata URLs rather than /download (which builds the
  whole route's tar.gz in device RAM and names members by opaque local_id).
  Returns (n_downloaded, n_cached).
  """
  get = (session or requests).get
  dest_dir = Path(dest_dir)
  fetched = cached = 0
  for seg, url in enumerate(log_urls):
    if not url:
      continue
    out = dest_dir / f"{date}--{seg}" / "rlog.zst"
    if out.exists() and out.stat().st_size > 0:
      cached += 1
      continue
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".part")
    with get(url, stream=True, timeout=300) as resp:
      resp.raise_for_status()
      with open(tmp, "wb") as f:
        for chunk in resp.iter_content(1 << 20):
          f.write(chunk)
    tmp.rename(out)
    fetched += 1
  return fetched, cached


def main():
  p = argparse.ArgumentParser()
  p.add_argument("--host", default=None, help="override `ssh -G c3` resolution")
  p.add_argument("--port", type=int, default=DEFAULT_PORT)
  p.add_argument("--limit", type=int, default=100, help="routes to enumerate")
  p.add_argument("--max-routes", type=int, default=None,
                 help="stop after downloading this many routes (newest first)")
  p.add_argument("--min-engagement", type=int, default=1)
  p.add_argument("--route", action="append", default=None,
                 help="route date string, repeatable; bypasses engagement filter")
  args = p.parse_args()

  host = args.host or c3_host()
  base = f"http://{host}:{args.port}"
  try:
    devices = requests.get(f"{base}/v1/me/devices/", timeout=10).json()
  except requests.ConnectionError as e:
    raise SystemExit(f"COD unreachable at {base} — is the C3 on and awake? "
                     f"(check `ssh c3`)\n{e}")
  dongle = devices[0]["dongle_id"]
  routes = requests.get(f"{base}/v1/devices/{dongle}/routes",
                        params={"limit": args.limit}, timeout=120).json()

  ROUTES_DIR.mkdir(parents=True, exist_ok=True)
  done = 0
  for r in routes:  # API returns newest first; keep that order
    date = r["fullname"].split("/")[1]
    if args.route is not None and date not in args.route:
      continue
    pct = r.get("engagement_pct") or 0
    if args.route is None and pct < args.min_engagement:
      print(f"skip {date}: engagement {pct}%")
      continue
    if args.max_routes is not None and done >= args.max_routes:
      print(f"stopping: --max-routes {args.max_routes} reached")
      break
    name = route_url_name(r["fullname"])
    log_urls = requests.get(f"{base}/v1/route/{name}/files", timeout=120) \
                       .json().get("logs", [])
    print(f"{date} (engagement {pct}%, {len(log_urls)} segments)...")
    fetched, cached = download_segments(log_urls, date, ROUTES_DIR)
    print(f"  {fetched} downloaded, {cached} already cached")
    done += 1


if __name__ == "__main__":
  main()
