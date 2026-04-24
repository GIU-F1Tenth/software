import argparse
import math
from dataclasses import dataclass
from itertools import combinations


@dataclass
class Point:
    x: float
    y: float


@dataclass
class Vector:
    tail: Point
    head: Point

    @property
    def dx(self) -> float:
        return self.head.x - self.tail.x

    @property
    def dy(self) -> float:
        return self.head.y - self.tail.y


def dot(v1: Vector, v2: Vector) -> float:
    return v1.dx * v2.dx + v1.dy * v2.dy


def euclidean(p1: Point, p2: Point) -> float:
    return math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)


def load_points(csv_path: str) -> list[Point]:
    points = []
    with open(csv_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            points.append(Point(float(parts[0]), float(parts[1])))
    return points


def build_vectors(points: list[Point]) -> list[Vector]:
    return [Vector(points[i], points[i + 1]) for i in range(len(points) - 1)]


def find_max_lookahead(vectors: list[Vector], tolerance: float) -> float:
    min_distance = float("inf")
    for v1, v2 in combinations(vectors, 2):
        if dot(v1, v2) < -tolerance:
            min_distance = min(min_distance, euclidean(v1.tail, v2.head))
    return min_distance


def main():
    parser = argparse.ArgumentParser(
        description="Find the maximum safe lookahead distance for pure pursuit"
    )
    parser.add_argument(
        "--csv-path", help="Path to the path CSV file (format: x_m, y_m, v_m/s)", required=True,
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.0,
        help="Dot product threshold to consider vectors as opposite direction",
    )
    args = parser.parse_args()

    if args.tolerance < 0:
        print("Tolerance must be non-negative.")
        return

    points = load_points(args.csv_path)
    vectors = build_vectors(points)
    max_lookahead = find_max_lookahead(vectors, args.tolerance)

    if math.isinf(max_lookahead):
        print("No opposite-direction vector pairs found.")
    else:
        print(f"Maximum lookahead distance: {max_lookahead:.4f} m")


if __name__ == "__main__":
    main()
