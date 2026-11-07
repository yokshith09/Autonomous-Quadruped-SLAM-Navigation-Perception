"""Spatial memory: facts an agent keeps about places, looked up by position in constant time.

Each agent owns one (frontier: retired frontiers; supervisor: doorways and objects). Items are bucketed on a square
hash grid, so `near()` reads only the buckets that overlap the query circle: cost depends on the query radius, not on
how much has been remembered over the run. Every item also carries the time it was stored, so the memory is
searchable by place, kind and time (the indexing eMEM, arXiv 2606.03374, argues embodied memory needs).
"""
import math
from collections import defaultdict
from typing import Any, Dict, Iterator, List, Optional, Tuple


class SpatialMemory:
    def __init__(self, bucket: float = 1.0):
        self.bucket = bucket
        self._cells: Dict[Tuple[int, int], List[Dict[str, Any]]] = defaultdict(list)
        self.count = 0

    def _key(self, x: float, y: float) -> Tuple[int, int]:
        return int(math.floor(x / self.bucket)), int(math.floor(y / self.bucket))

    def add(self, kind: str, x: float, y: float, t: float = 0.0, **data) -> Dict[str, Any]:
        item = {"kind": kind, "x": float(x), "y": float(y), "t": float(t), **data}
        self._cells[self._key(x, y)].append(item)
        self.count += 1
        return item

    def near(self, kind: Optional[str], x: float, y: float, r: float) -> Iterator[Dict[str, Any]]:
        """Items of `kind` (any kind if None) within distance r of (x, y)."""
        (x0, y0), (x1, y1) = self._key(x - r, y - r), self._key(x + r, y + r)
        r2 = r * r
        for i in range(x0, x1 + 1):
            for j in range(y0, y1 + 1):
                for it in self._cells.get((i, j), ()):
                    if (kind is None or it["kind"] == kind) and (it["x"] - x) ** 2 + (it["y"] - y) ** 2 <= r2:
                        yield it

    def any_near(self, kind: Optional[str], x: float, y: float, r: float) -> bool:
        return next(self.near(kind, x, y, r), None) is not None

    def nearest(self, kind: Optional[str], x: float, y: float, r: float) -> Optional[Dict[str, Any]]:
        return min(self.near(kind, x, y, r), key=lambda it: (it["x"] - x) ** 2 + (it["y"] - y) ** 2, default=None)

    def items(self, kind: Optional[str] = None) -> List[Dict[str, Any]]:
        return [it for cell in self._cells.values() for it in cell if kind is None or it["kind"] == kind]
