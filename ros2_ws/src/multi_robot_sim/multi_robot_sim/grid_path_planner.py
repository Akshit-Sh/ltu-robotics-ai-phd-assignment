"""A* path planning on a grid rasterised from an SDF world."""

from dataclasses import dataclass
import heapq
import math
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class BoxObstacle:
    """An axis-aligned-size box in world coordinates."""

    x: float
    y: float
    size_x: float
    size_y: float
    yaw: float = 0.0


class AStarGridPlanner:
    """Plan paths around inflated rectangular SDF collision geometry."""

    def __init__(
        self,
        obstacles,
        resolution=0.10,
        bounds=(-3.8, 3.8, -3.8, 3.8),
        inflation=0.22,
    ):
        if resolution <= 0.0:
            raise ValueError("Grid resolution must be positive.")
        if inflation < 0.0:
            raise ValueError("Obstacle inflation cannot be negative.")

        self.obstacles = tuple(obstacles)
        self.resolution = float(resolution)
        self.bounds = tuple(float(v) for v in bounds)
        self.inflation = float(inflation)

        self.x_min, self.x_max, self.y_min, self.y_max = self.bounds
        if self.x_max <= self.x_min or self.y_max <= self.y_min:
            raise ValueError("Grid bounds must have positive dimensions.")

        self.width = int(round(
            (self.x_max - self.x_min) / self.resolution
        )) + 1
        self.height = int(round(
            (self.y_max - self.y_min) / self.resolution
        )) + 1

        self._blocked = [
            [
                self._point_is_blocked(
                    self.x_min + ix * self.resolution,
                    self.y_min + iy * self.resolution,
                )
                for ix in range(self.width)
            ]
            for iy in range(self.height)
        ]

    @classmethod
    def from_sdf(
        cls,
        sdf_path,
        resolution=0.10,
        bounds=(-3.8, 3.8, -3.8, 3.8),
        inflation=0.22,
    ):
        """Extract box collision geometry from a local SDF world."""
        root = ET.parse(str(sdf_path)).getroot()
        world = root.find("world")
        if world is None:
            raise ValueError("SDF file does not contain a world element.")

        obstacles = []

        for model in world.findall("model"):
            model_pose = cls._parse_pose(model.findtext("pose"))
            mx, my, _, _, _, myaw = model_pose

            for collision in model.findall("./link/collision"):
                size_text = collision.findtext("geometry/box/size")
                if not size_text:
                    continue

                size = [float(v) for v in size_text.split()]
                if len(size) != 3:
                    raise ValueError("SDF box size must have three values.")

                pose = cls._parse_pose(collision.findtext("pose"))
                cx, cy, _, _, _, cyaw = pose

                x = mx + math.cos(myaw) * cx - math.sin(myaw) * cy
                y = my + math.sin(myaw) * cx + math.cos(myaw) * cy

                obstacles.append(
                    BoxObstacle(
                        x=x,
                        y=y,
                        size_x=size[0],
                        size_y=size[1],
                        yaw=myaw + cyaw,
                    )
                )

        if not obstacles:
            raise ValueError("No box collision geometry found in SDF.")

        return cls(
            obstacles,
            resolution=resolution,
            bounds=bounds,
            inflation=inflation,
        )

    @staticmethod
    def _parse_pose(text):
        """Return the six SDF pose values, defaulting to the origin."""
        if not text:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        values = [float(v) for v in text.split()]
        if len(values) != 6:
            raise ValueError("SDF pose must have six values.")
        return tuple(values)

    def world_to_grid(self, x, y):
        """Convert world coordinates to the nearest grid cell."""
        ix = round((x - self.x_min) / self.resolution)
        iy = round((y - self.y_min) / self.resolution)
        if not (0 <= ix < self.width and 0 <= iy < self.height):
            return None
        return int(ix), int(iy)

    def grid_to_world(self, cell):
        """Return the world-space centre of a grid cell."""
        ix, iy = cell
        return (
            self.x_min + ix * self.resolution,
            self.y_min + iy * self.resolution,
        )

    def _point_is_blocked(self, x, y):
        """Check a point against boxes inflated by the safety margin."""
        for obstacle in self.obstacles:
            dx = x - obstacle.x
            dy = y - obstacle.y
            c = math.cos(obstacle.yaw)
            s = math.sin(obstacle.yaw)
            local_x = c * dx + s * dy
            local_y = -s * dx + c * dy

            half_x = obstacle.size_x / 2.0 + self.inflation
            half_y = obstacle.size_y / 2.0 + self.inflation
            if abs(local_x) <= half_x and abs(local_y) <= half_y:
                return True
        return False

    def is_blocked(self, cell):
        """Return true for occupied cells or cells outside the grid."""
        ix, iy = cell
        if not (0 <= ix < self.width and 0 <= iy < self.height):
            return True
        return self._blocked[iy][ix]

    def _heuristic(self, a, b):
        """Octile-distance heuristic for eight-connected movement."""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        diagonal = min(dx, dy)
        straight = max(dx, dy) - diagonal
        return diagonal * math.sqrt(2.0) + straight

    def _neighbors(self, cell):
        """Yield reachable neighbors without cutting obstacle corners."""
        ix, iy = cell
        for dx, dy in (
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ):
            neighbor = (ix + dx, iy + dy)
            if self.is_blocked(neighbor):
                continue

            if dx and dy:
                if self.is_blocked((ix + dx, iy)):
                    continue
                if self.is_blocked((ix, iy + dy)):
                    continue

            cost = math.sqrt(2.0) if dx and dy else 1.0
            yield neighbor, cost

    def _segment_is_free(self, start, end):
        """Sample a line segment to keep smoothed paths in free space."""
        x0, y0 = self.grid_to_world(start)
        x1, y1 = self.grid_to_world(end)
        distance = math.hypot(x1 - x0, y1 - y0)
        step = self.resolution / 3.0
        samples = max(1, math.ceil(distance / step))

        for index in range(samples + 1):
            ratio = index / samples
            x = x0 + ratio * (x1 - x0)
            y = y0 + ratio * (y1 - y0)
            cell = self.world_to_grid(x, y)
            if cell is None or self.is_blocked(cell):
                return False
        return True

    def _simplify(self, cells):
        """Skip intermediate cells only when the line remains clear."""
        if len(cells) <= 2:
            return cells

        result = [cells[0]]
        anchor = 0
        while anchor < len(cells) - 1:
            candidate = len(cells) - 1
            while candidate > anchor + 1:
                if self._segment_is_free(cells[anchor], cells[candidate]):
                    break
                candidate -= 1
            result.append(cells[candidate])
            anchor = candidate
        return result

    def plan(self, start, goal):
        """Return a smoothed world-coordinate path, or None if unreachable."""
        start_cell = self.world_to_grid(*start)
        goal_cell = self.world_to_grid(*goal)

        if start_cell is None or self.is_blocked(start_cell):
            raise ValueError("Start is outside the grid or in an obstacle.")
        if goal_cell is None or self.is_blocked(goal_cell):
            raise ValueError("Goal is outside the grid or in an obstacle.")
        if start_cell == goal_cell:
            return [self.grid_to_world(start_cell)]

        frontier = []
        heapq.heappush(frontier, (
            self._heuristic(start_cell, goal_cell), 0.0, start_cell
        ))
        came_from = {}
        cost_so_far = {start_cell: 0.0}
        closed = set()

        while frontier:
            _, cost, current = heapq.heappop(frontier)
            if current in closed:
                continue
            if current == goal_cell:
                break
            closed.add(current)

            for neighbor, step_cost in self._neighbors(current):
                new_cost = cost + step_cost
                if new_cost >= cost_so_far.get(neighbor, math.inf):
                    continue
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + self._heuristic(
                    neighbor, goal_cell
                )
                heapq.heappush(
                    frontier, (priority, new_cost, neighbor)
                )
        else:
            return None

        cells = [goal_cell]
        while cells[-1] != start_cell:
            cells.append(came_from[cells[-1]])
        cells.reverse()

        return [
            self.grid_to_world(cell)
            for cell in self._simplify(cells)
        ]
