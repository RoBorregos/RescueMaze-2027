#include "maze.h"
#include "config.h"

namespace {
// The 2026 DFS pushes left, back, right, front in map coordinates.
constexpr uint8_t kDirectionOrder[] = {3, 2, 1, 0};

uint8_t tileCost(RescueMaze::TileType type) {
    switch (type) {
        case RescueMaze::TileType::kBlue: return MazeConfig::kBlueTileCost;
        case RescueMaze::TileType::kBlack: return MazeConfig::kBlackTileCost;
        case RescueMaze::TileType::kRamp: return MazeConfig::kRampTileCost;
        case RescueMaze::TileType::kWhite: return MazeConfig::kWhiteTileCost;
    }
    return MazeConfig::kWhiteTileCost;
}
}

uint8_t RescueMaze::findTile(int16_t x, int16_t y) const {
    for (uint8_t i = 0; i < count; ++i) {
        if (tiles[i].x == x && tiles[i].y == y) return i;
    }
    return kMissing;
}

void RescueMaze::removePending(uint8_t tile) {
    for (uint8_t i = 0; i < pendingCount; ++i) {
        if (pending[i] != tile) continue;
        for (uint8_t j = i + 1; j < pendingCount; ++j) pending[j - 1] = pending[j];
        --pendingCount;
        return;
    }
}

bool RescueMaze::observe(uint8_t openDirections, uint8_t measuredDirections) {
    constexpr int8_t dx[] = {0, 1, 0, -1};
    constexpr int8_t dy[] = {1, 0, -1, 0};
    Tile &tile = tiles[current];
    for (uint8_t direction : kDirectionOrder) {
        const uint8_t bit = 1u << direction;
        if (!(measuredDirections & bit)) continue;
        const int16_t x = tile.x + dx[direction];
        const int16_t y = tile.y + dy[direction];
        uint8_t neighbor = findTile(x, y);
        const uint8_t opposite = (direction + 2) % 4;
        const uint8_t oppositeBit = 1u << opposite;
        if (!(openDirections & bit)) {
            tile.walls |= bit;
            tile.neighbor[direction] = kMissing;
            if (neighbor != kMissing) {
                tiles[neighbor].walls |= oppositeBit;
                tiles[neighbor].neighbor[opposite] = kMissing;
            }
            continue;
        }
        if (neighbor == kMissing) {
            if (count == kCapacity) return false;
            neighbor = count++;
            tiles[neighbor].x = x;
            tiles[neighbor].y = y;
        }
        if (tile.neighbor[direction] == kMissing && !tiles[neighbor].visited) {
            // Keep the newest entry. Older duplicates would be skipped after visiting it.
            removePending(neighbor);
            pending[pendingCount++] = neighbor;
        }
        tile.walls &= ~bit;
        tiles[neighbor].walls &= ~oppositeBit;
        tile.neighbor[direction] = neighbor;
        tiles[neighbor].neighbor[opposite] = current;
    }
    tile.visited = true;
    removePending(current);
    return true;
}

bool RescueMaze::setTileType(int16_t x, int16_t y, TileType type) {
    const uint8_t tile = findTile(x, y);
    if (tile == kMissing) return false;
    tiles[tile].type = type;
    return true;
}

RescueMaze::Step RescueMaze::next(uint8_t &direction, uint8_t skippedDirections) const {
    if (!tiles[current].visited) return Step::kNoRoute;

    // Each transition pays the cost of entering the destination tile.
    constexpr uint16_t kUnreachable = UINT16_MAX;
    uint16_t distances[kCapacity];
    uint8_t firstDirection[kCapacity] = {};
    bool settled[kCapacity] = {};
    for (uint8_t i = 0; i < count; ++i) distances[i] = kUnreachable;
    distances[current] = 0;
    while (true) {
        uint8_t tile = kMissing;
        uint16_t nearest = kUnreachable;
        for (uint8_t i = 0; i < count; ++i) {
            if (settled[i] || distances[i] >= nearest) continue;
            nearest = distances[i];
            tile = i;
        }
        if (tile == kMissing) break;
        settled[tile] = true;
        if (!tiles[tile].visited) continue;
        for (uint8_t side : kDirectionOrder) {
            if (tile == current && (skippedDirections & (1u << side))) continue;
            const uint8_t neighbor = tiles[tile].neighbor[side];
            if (neighbor == kMissing) continue;
            const uint16_t distance = nearest + tileCost(tiles[neighbor].type);
            if (distance >= distances[neighbor]) continue;
            distances[neighbor] = distance;
            firstDirection[neighbor] = tile == current ? side : firstDirection[tile];
        }
    }
    // A new wall may isolate a pending branch. Keep it for a possible reopening.
    for (uint8_t i = pendingCount; i > 0; --i) {
        const uint8_t candidate = pending[i - 1];
        if (distances[candidate] != kUnreachable) {
            direction = firstDirection[candidate];
            return Step::kMove;
        }
    }
    if (current == 0) return skippedDirections == 0 ? Step::kDone : Step::kNoRoute;
    if (distances[0] == kUnreachable) return Step::kNoRoute;
    direction = firstDirection[0];
    return Step::kMove;
}

bool RescueMaze::arrive(uint8_t direction) {
    if (direction >= 4 || tiles[current].neighbor[direction] == kMissing) return false;
    current = tiles[current].neighbor[direction];
    return true;
}

uint8_t RescueMaze::knownOpenDirections() const {
    uint8_t open = 0;
    for (uint8_t side = 0; side < 4; ++side) {
        if (tiles[current].neighbor[side] != kMissing) open |= 1u << side;
    }
    return open;
}
