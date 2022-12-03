#include "raylib.h"
#include <functional>
#include <vector>
#include <limits>
#include <cmath>
#include "math.h"
#include "dungeonGen.h"
#include "dungeonUtils.h"
#include <iostream>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <utility>

template<typename T>
static size_t coord_to_idx(T x, T y, size_t w)
{
  return size_t(y) * w + size_t(x);
}

static void draw_nav_grid(const char *input, size_t width, size_t height)
{
  for (size_t y = 0; y < height; ++y)
    for (size_t x = 0; x < width; ++x)
    {
      char symb = input[coord_to_idx(x, y, width)];
      Color color = GetColor(symb == ' ' ? 0xeeeeeeff : symb == 'o' ? 0x7777ffff : 0x222222ff);
      DrawPixel(int(x), int(y), color);
    }
}

static void draw_path(std::vector<Position> path)
{
  for (const Position &p : path)
    DrawPixel(p.x, p.y, GetColor(0x44000088));
}

static std::vector<Position> reconstruct_path(std::vector<Position> prev, Position to, size_t width)
{
  Position curPos = to;
  std::vector<Position> res = {curPos};
  while (prev[coord_to_idx(curPos.x, curPos.y, width)] != Position{-1, -1})
  {
    curPos = prev[coord_to_idx(curPos.x, curPos.y, width)];
    res.push_back(curPos);
  }
  return res;
}


namespace std
{
    template<> struct hash<Position>
    {
        using argument_type = Position;
        using result_type = std::size_t;
        result_type operator()(argument_type const& a) const
        {
            result_type const h1(std::hash<int>()(a.x));
            result_type const h2(std::hash<int>()(a.y));
            return h1 ^ (h2 << 1);
        }
    };
}

struct PositionPriority
{
    Position pos;
    float priority;
};

class PrioritySet 
{
public:
    PrioritySet() {};

    template<class T, class...Ts>
    PrioritySet(T a, Ts... meh)
    {
        (push(a), ..., push(meh));
    }

    void push(Position pos) {
        openList.push({pos, 0});
        stillOpen.insert(pos);
    }

    void push(PositionPriority pair) {
        openList.push(pair);
        stillOpen.insert(pair.pos);
    }

    PositionPriority pop()
    {
        auto cur = openList.top();
        openList.pop();
        stillOpen.erase(stillOpen.find(cur.pos));
        return cur;
    }

    bool empty() { return openList.empty(); }

    bool contains(Position pos) 
    {
        return stillOpen.contains(pos);
    }

private:
    constexpr static auto cmp = [](PositionPriority left, PositionPriority right) { return (left.priority) > (right.priority); };
    std::priority_queue<PositionPriority, std::vector<PositionPriority>, decltype(cmp)> openList{ cmp };
    std::unordered_set<Position> stillOpen {};
};

static std::vector<Position> find_ara_star(const char* input, size_t width, size_t height, Position from, Position to)
{
    if (from.x < 0 || from.y < 0 || from.x >= int(width) || from.y >= int(height))
        return std::vector<Position>();
    size_t inpSize = width * height;
    
    return {};
}

class AStarPathfinder
{
public:
    AStarPathfinder(size_t width, size_t height) :
        width(width)
        , height(height)
        , input_size(width* height)
        , g(input_size, std::numeric_limits<float>::max())
        , f(input_size, std::numeric_limits<float>::max())
        , prev(input_size, { -1,-1 })
    {}

    std::vector<Position> find_path_a_star(const char* input, Position from, Position to)
    {
        if (from.x < 0 || from.y < 0 || from.x >= int(width) || from.y >= int(height))
            return std::vector<Position>();

        if (prev_from == from && prev_to == to)
        {
            display_weights();
            return reconstruct_path(prev, to, width);
        }
        prev_from = from;
        prev_to = to;
        clear();

        return pathfind(input, from, to);
    }

private:
    std::vector<Position> pathfind(const char* input, const Position& from, const Position& to)
    {
        size_t id = coord_to_idx(from.x, from.y, width);
        g[id] = 0;
        f[id] = heuristic(from, to);

        PrioritySet meh{ from };
        std::unordered_set<Position> closedList;

        while (!meh.empty())
        {
            Position curPos = meh.pop().pos;
            if (curPos == to)
            {
                display_weights();
                return reconstruct_path(prev, to, width);
            }
            if (closedList.contains(curPos))
                continue;

            display.push_back(curPos);

            closedList.insert(curPos);
            auto checkNeighbour = [&](Position p)
            {
                // out of bounds
                if (p.x < 0 || p.y < 0 || p.x >= int(width) || p.y >= int(height))
                    return;
                size_t idx = coord_to_idx(p.x, p.y, width);
                // not empty
                if (input[idx] == '#')
                    return;

                float weight = input[idx] == 'o' ? 10.f : 1.f;
                float gScore = getG(curPos) + 1.f * weight; // we're exactly 1 unit away
                if (gScore < getG(p))
                {
                    prev[idx] = curPos;
                    g[idx] = gScore;
                    f[idx] = gScore + heuristic(p, to);
                }

                if (!meh.contains(p))
                {
                    meh.push({ p, f[idx] });
                }
            };
            checkNeighbour({ curPos.x + 1, curPos.y + 0 });
            checkNeighbour({ curPos.x - 1, curPos.y + 0 });
            checkNeighbour({ curPos.x + 0, curPos.y + 1 });
            checkNeighbour({ curPos.x + 0, curPos.y - 1 });
        }
        return std::vector<Position>();
    }

    void display_weights() 
    {
        for (const auto& pos : display)
        {
            size_t idx = coord_to_idx(pos.x, pos.y, width);
            DrawPixel(pos.x, pos.y, Color{ uint8_t(g[idx]), uint8_t(g[idx]), 0, 100 });
        }
    }

    void clear() 
    {
        std::fill(prev.begin(), prev.end(), Position{ -1,-1 });
        std::fill(g.begin(), g.end(), std::numeric_limits<float>::max());
        std::fill(f.begin(), f.end(), std::numeric_limits<float>::max());
        display.clear();
    }

    float getG(const Position& p) { return g[coord_to_idx(p.x, p.y, width)]; }
    float getF(const Position& p) { return f[coord_to_idx(p.x, p.y, width)]; }
    float heuristic(const Position& lhs, const Position& rhs) {
        return sqrtf(square(float(lhs.x - rhs.x)) + square(float(lhs.y - rhs.y)));
    }

private:
    size_t width{};
    size_t height{};
    size_t input_size{};

    std::vector<float> g;
    std::vector<float> f;
    std::vector<Position> prev;
    Position prev_from = { -1, -1 };
    Position prev_to = { -1, -1 };
    std::vector<Position> display{};
};

void draw_nav_data(const char *input, size_t width, size_t height, Position from, Position to)
{
  draw_nav_grid(input, width, height);
  static AStarPathfinder finder(width, height);
  std::vector<Position> path = finder.find_path_a_star(input, from, to);
  draw_path(path);
}

int main(int /*argc*/, const char ** /*argv*/)
{
  int width = 1480;
  int height = 720;
  InitWindow(width, height, "w3 AI MIPT");

  const int scrWidth = GetMonitorWidth(0);
  const int scrHeight = GetMonitorHeight(0);
  if (scrWidth < width || scrHeight < height)
  {
    width = std::min(scrWidth, width);
    height = std::min(scrHeight - 150, height);
    SetWindowSize(width, height);
  }

  constexpr size_t dungWidth = 200;
  constexpr size_t dungHeight = 100;
  char *navGrid = new char[dungWidth * dungHeight];
  gen_drunk_dungeon(navGrid, dungWidth, dungHeight, 24, 100);
  spill_drunk_water(navGrid, dungWidth, dungHeight, 8, 10);

  Position from = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
  Position to = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);

  Camera2D camera = { {0, 0}, {0, 0}, 0.f, 1.f };
  //camera.offset = Vector2{ width * 0.5f, height * 0.5f };
  camera.zoom = float(height) / float(dungHeight);

  SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
  while (!WindowShouldClose())
  {
    // pick pos
    Vector2 mousePosition = GetScreenToWorld2D(GetMousePosition(), camera);
    Position p{int(mousePosition.x), int(mousePosition.y)};
    if (IsMouseButtonPressed(2) || IsKeyPressed(KEY_Q))
    {
      size_t idx = coord_to_idx(p.x, p.y, dungWidth);
      if (idx < dungWidth * dungHeight)
        navGrid[idx] = navGrid[idx] == ' ' ? '#' : navGrid[idx] == '#' ? 'o' : ' ';
    }
    else if (IsMouseButtonPressed(0))
    {
      Position &target = from;
      target = p;
    }
    else if (IsMouseButtonPressed(1))
    {
      Position &target = to;
      target = p;
    }
    if (IsKeyPressed(KEY_SPACE))
    {
      gen_drunk_dungeon(navGrid, dungWidth, dungHeight, 24, 100);
      spill_drunk_water(navGrid, dungWidth, dungHeight, 8, 10);
      from = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
      to = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
    }
    BeginDrawing();
      ClearBackground(BLACK);
      BeginMode2D(camera);
        draw_nav_data(navGrid, dungWidth, dungHeight, from, to);
      EndMode2D();
    EndDrawing();
  }
  CloseWindow();
  return 0;
}
