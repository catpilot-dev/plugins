from fetch_routes import route_url_name, download_segments


class _Resp:
  def __init__(self, payload):
    self.payload = payload

  def raise_for_status(self):
    pass

  def iter_content(self, _size):
    yield self.payload

  def __enter__(self):
    return self

  def __exit__(self, *a):
    return False


class _Session:
  def __init__(self):
    self.urls = []

  def get(self, url, **_kw):
    self.urls.append(url)
    return _Resp(b"fake:" + url.encode())


def test_route_url_name_swaps_separator_and_quotes():
  assert route_url_name("abc123/2026-02-20--10-47-46") == \
      "abc123%7C2026-02-20--10-47-46"


def test_download_segments_normalizes_layout(tmp_path):
  urls = ["http://d/0/rlog.zst", "", "http://d/2/rlog.zst"]  # seg 1 unavailable
  sess = _Session()
  assert download_segments(urls, "2026-02-20--10-47-46", tmp_path, sess) == (2, 0)
  assert (tmp_path / "2026-02-20--10-47-46--0" / "rlog.zst").read_bytes() == \
      b"fake:http://d/0/rlog.zst"
  assert (tmp_path / "2026-02-20--10-47-46--2" / "rlog.zst").exists()
  assert not (tmp_path / "2026-02-20--10-47-46--1").exists()
  assert not list(tmp_path.glob("**/*.part"))


def test_download_segments_skips_cached(tmp_path):
  urls = ["http://d/0/rlog.zst", "http://d/1/rlog.zst"]
  seg0 = tmp_path / "2026-02-20--10-47-46--0"
  seg0.mkdir(parents=True)
  (seg0 / "rlog.zst").write_bytes(b"already here")
  sess = _Session()
  assert download_segments(urls, "2026-02-20--10-47-46", tmp_path, sess) == (1, 1)
  assert sess.urls == ["http://d/1/rlog.zst"]
  assert (seg0 / "rlog.zst").read_bytes() == b"already here"
