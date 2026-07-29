import io
import tarfile

from fetch_routes import route_url_name, extract_rlogs


def test_route_url_name_swaps_separator_and_quotes():
  assert route_url_name("abc123/2026-02-20--10-47-46") == \
      "abc123%7C2026-02-20--10-47-46"


def test_extract_rlogs_normalizes_layout(tmp_path):
  tar_path = tmp_path / "dl.tar.gz"
  with tarfile.open(tar_path, "w:gz") as tf:
    for name in ("2026-02-20--10-47-46--0/rlog.zst",
                 "2026-02-20--10-47-46--1/rlog.zst",
                 "2026-02-20--10-47-46--0/qlog.zst"):   # non-rlog ignored
      data = b"fake"
      info = tarfile.TarInfo(name)
      info.size = len(data)
      tf.addfile(info, io.BytesIO(data))
  dest = tmp_path / "routes"
  assert extract_rlogs(tar_path, dest) == 2
  assert (dest / "2026-02-20--10-47-46--0" / "rlog.zst").read_bytes() == b"fake"
  assert not (dest / "2026-02-20--10-47-46--0" / "qlog.zst").exists()
