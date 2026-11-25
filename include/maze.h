#ifndef RESCUE_MAZE_H
#define RESCUE_MAZE_H

#include <stdint.h>

// Planar exploration from Rescue Maze 2026: DFS with shortest-path transit.
// Direction indices increase clockwise, starting with the initial heading.
class RescueMaze {
public:
    enum class Step { kMove, kDone, kNoRoute };
    enum class TileType : uint8_t { kWhite, kBlue, kBlack, kRamp };

    bool observe(uint8_t openDirections, uint8_t measuredDirections = 0x0F);
    bool setTileType(int16_t x, int16_t y, TileType type);
    Step next(uint8_t &direction, uint8_t skippedDirections = 0) const;
    bool arrive(uint8_t direction);
    uint8_t knownOpenDirections() const;
    uint8_t knownWallDirections() const { return tiles[current].walls; }
    int16_t x() const { return tiles[current].x; }
    int16_t y() const { return tiles[current].y; }

private:
    static constexpr uint8_t kCapacity = 70;
    static constexpr uint8_t kMissing = 255;
    struct Tile {
        int16_t x = 0;
        int16_t y = 0;
        uint8_t neighbor[4] = {kMissing, kMissing, kMissing, kMissing};
        uint8_t walls = 0;
        bool visited = false;
        TileType type = TileType::kWhite;
    };

    Tile tiles[kCapacity];
    uint8_t pending[kCapacity];
    uint8_t count = 1;
    uint8_t current = 0;
    uint8_t pendingCount = 0;

    uint8_t findTile(int16_t x, int16_t y) const;
    void removePending(uint8_t tile);
};

#endif
