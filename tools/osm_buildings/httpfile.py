"""A seekable read-only file over HTTP range requests (urllib honours HTTPS_PROXY), with a block cache."""
import io, urllib.request

class HttpFile(io.RawIOBase):
    def __init__(self, url, block=1 << 20):
        self.url, self.pos, self.block, self.cache = url, 0, block, {}
        req = urllib.request.Request(url, method='HEAD')
        with urllib.request.urlopen(req, timeout=60) as r:
            self.size = int(r.headers['Content-Length'])
        self.fetched = 0
    def readable(self): return True
    def seekable(self): return True
    def tell(self): return self.pos
    def seek(self, off, whence=0):
        self.pos = off if whence == 0 else self.pos + off if whence == 1 else self.size + off
        return self.pos
    def _get(self, a, b):
        req = urllib.request.Request(self.url, headers={'Range': f'bytes={a}-{b - 1}'})
        for attempt in range(5):
            try:
                with urllib.request.urlopen(req, timeout=120) as r:
                    d = r.read()
                self.fetched += len(d)
                return d
            except Exception:
                if attempt == 4: raise
    def read(self, n=-1):
        if n is None or n < 0: n = self.size - self.pos
        n = min(n, self.size - self.pos)
        if n <= 0: return b''
        if n > 4 * self.block:  # large reads straight through
            d = self._get(self.pos, self.pos + n); self.pos += n; return d
        out = bytearray()
        while n > 0:
            bi = self.pos // self.block
            if bi not in self.cache:
                a = bi * self.block
                self.cache[bi] = self._get(a, min(a + self.block, self.size))
            blk = self.cache[bi]; o = self.pos - bi * self.block
            take = blk[o:o + n]; out += take; self.pos += len(take); n -= len(take)
        return bytes(out)
    def readinto(self, b):
        d = self.read(len(b)); b[:len(d)] = d; return len(d)
