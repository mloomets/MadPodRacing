#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>
#include <memory>
#include <algorithm>

// #define LOG

constexpr int maxAbsAngle = 180;
constexpr int maxThrust = 100;
constexpr int checkpointSize = 650;


constexpr double vectorComparisonPrecision = 0.001;

constexpr double goalComparisonPrecision = 0.001;

constexpr int movementCorrectionWeight = 500;

constexpr int attackTargetLocationOffset = 3;

constexpr int maxAngleForBoost = 20;

constexpr int minDisForBoost = 6500;

constexpr int minSpeedForShield = 350;

constexpr int angleMultThreshold = 70;

constexpr int enemyDistMultRange = 2000;

constexpr double distMultRange = 2645.0;

class Vector;
class Pod;
class Goal;
struct Progress;

std::vector<Goal> goals;
std::vector<Vector> nextPoint;
std::unique_ptr<std::pair<Pod, Pod> > pods{};
std::unique_ptr<std::pair<Pod, Pod> > enemyPods{};
Pod * bestEnemy;
int laps;
bool boostUsed = false;

static double degToRad(const double deg) {
    return deg * M_PI / 180.0;
}

static double radToDeg(const double rad) {
    return rad * 180 / M_PI;
}

struct Progress{
    int lap = 1;
    int goal = 0;
    double distanceToNext = -1;
};

class Goal {
public:
    std::pair<int, int> _adjustedTarget;

    Goal(): _x(0), _y(0), _distNext(0), _next(nullptr), _prev(nullptr) {}

    Goal(int x, int y): _x(x), _y(y), _distNext(0), _next(nullptr), _prev(nullptr) {}

    Goal(int x, int y, int distNext): _x(x), _y(y), _distNext(distNext), _next(nullptr), _prev(nullptr) {}

    Goal(int x, int y, Goal* next):
        _x(x),
        _y(y),
        _distNext(distanceTo(next->_x, next->_y)),
        _next(next),
        _prev(nullptr)
    {}

    [[nodiscard]] std::string toString() const {
        return "[" + std::to_string(_x) + "," + std::to_string(_y) + "," + std::to_string(_distNext) + "]";
    }

    [[nodiscard]] double distanceTo(double x, double y) const {
        return sqrt(pow(_x - x, 2) + pow(_y - y, 2));
    }

    void setNextGoal(const std::shared_ptr<Goal>& next) {
        _next = next;
        _distNext = distanceTo(next->_x, next->_y);
        if (next->_prev != std::shared_ptr<Goal>(this)) {
            next->_prev = std::shared_ptr<Goal>(this);
        }
    }

    [[nodiscard]] std::string toStringNext() const {
        return "{" + _next->toString() + ", " + std::to_string(_distNext) + "}";
    }

    void setX(double x) {
        _x = x;
    }

    void setY(double y) {
        _y = y;
    }

    void setXY(double x, double y) {
        setX(x);
        setY(y);
    }

    [[nodiscard]] double getX() const {
        return _x;
    }

    [[nodiscard]] double getY() const {
        return _y;
    }

    void setPrevGoal(const std::shared_ptr<Goal>& prev) {
        _prev = prev;
        prev->setNextGoal(std::shared_ptr<Goal>(this));
    }

    [[nodiscard]] std::shared_ptr<Goal> getNext() const {
        return _next;
    }

    bool operator>(const Goal& other) const {
        return _distNext > other._distNext;
    }

    bool operator<(const Goal& other) const {
        return  _distNext < other._distNext;
    }

    bool operator==(const Goal& other) const {
        return std::abs(_x - other._x) < goalComparisonPrecision &&
               std::abs(_y - other._y) < goalComparisonPrecision;
    }

    bool operator!=(const Goal& other) const {
        return !(std::abs(_x - other._x) < goalComparisonPrecision &&
                 std::abs(_y - other._y) < goalComparisonPrecision);
    }

private:
    double _x, _y, _distNext;
    std::shared_ptr<Goal> _next;
    std::shared_ptr<Goal> _prev;

};

class Vector {
public:
    Vector(): _x(0), _y(0) {}

    Vector(const int x, const int y): _x(x), _y(y) {}

    explicit Vector(const std::pair<int, int> vec): _x(vec.first), _y(vec.second) {}

    explicit Vector(const Goal& goal): _x(goal.getX()), _y(goal.getY()){}

    Vector(const double x, const double y): _x(x), _y(y) {}

    Vector(const int x1, const int y1, const int x2, const int y2): _x(x2 - x1), _y(y2 - y1) {}

    Vector(const double x1, const double y1, const double x2, const double y2): _x(x2 - x1), _y(y2 - y1) {}

    [[nodiscard]] std::string toString() const {
        return "(" + std::to_string(_x) + "," + std::to_string(_y) + ")";
    }

    [[nodiscard]] double len() const {
        return sqrt(pow(_x, 2) + pow(_y, 2));
    }

    void normalize() {
        Vector normalized = normalizedValue();
        _x = normalized._x;
        _y = normalized._y;
    }

    [[nodiscard]] Vector normalizedValue() const {
        double length = len();
        return length != 0.0 ? Vector(_x / length, _y / length): Vector(0, 0);

    }

    [[nodiscard]] Vector rotatedValue(double angle) const {
        double cosVal = cos(angle);
        double sinVal = cos(angle);
        return {_x * cosVal - _y * sinVal,
                _x * cosVal + _y * sinVal};
    }

    [[nodiscard]] std::pair<int, int> toPoint() const {
        return {_x, _y};
    }

    [[nodiscard]] double angleTo(const Vector& end) const {
        double angle = acos((*this * end) / (len() * end.len()));
        return fmod(radToDeg(angle), 360.0);
    }

    [[nodiscard]] double getX() const {
        return _x;
    }

    [[nodiscard]] double getY() const  {
        return _y;
    }

    Vector operator+(const Vector& rhs) const {
        return {_x + rhs._x, _y + rhs._y};
    }

    Vector operator+(const Goal& rhs) const {
        return {_x + rhs.getX(), _y + rhs.getY()};
    }

    Vector operator-(const Vector& rhs) const {
        return {_x - rhs._x, _y - rhs._y};
    }

    Vector operator*(const double& rhs) const {
        return {_x * rhs, _y * rhs};
    }

    double operator*(const Vector& rhs) const {
        return _x * rhs._x + _y * rhs._y;
    }

    Vector operator/(const double& rhs) const {
        return {_x * rhs, _y * rhs};
    }

    bool operator==(const Vector& rhs) const {
        return std::abs(_x - rhs._x) < vectorComparisonPrecision &&
               std::abs(_y - rhs._y) < vectorComparisonPrecision;
    }

private:
    double _x, _y;
};

class Pod {
public:
    Vector location;
    Vector speed;
    Vector calcSpeed;
    double angle;
    int nextGoalId;
    Progress progress;

    enum Strategy {RACE, ATTACK};
    Strategy strategy;

    Pod(): nextGoalId(1), angle(0.0), strategy(Strategy::RACE), podId(podCount++) {}

    [[nodiscard]] std::string toString() const {

        return "Pod" + std::to_string(podId) + " " + std::to_string(progress.lap) + " " +
               std::to_string(progress.goal) + " " + std::to_string(progress.distanceToNext) + " " +
               std::to_string(calcSpeed.len()) + " " + strategyToString(strategy);
    }

    void updateProgress() {
        if (nextGoalId == 1 && nextGoalId < progress.goal) {
            ++progress.lap;
        }
        progress.goal = static_cast<int>((nextGoalId - 1 + goals.size()) % goals.size());
        progress.distanceToNext = (location - Vector(goals[nextGoalId])).len();
    }

    std::string getAction() {
#ifdef LOG
        std::cerr << "Getting action of " << toString() << "\n";
        std::cerr << "Location " << location.toString() << "\n";
#endif
        Vector nextPointTarget = getTargetLocation();

#ifdef LOG
        std::cerr << "Done calculating nextPointTarget " << nextPointTarget.toString() << "\n";
#endif
        Vector target = getSpeedCorrectedTarget(nextPointTarget);
#ifdef LOG
        std::cerr << "Done calculating target " << target.toString() << "\n";
#endif
        double nextGoalDist = target.len();
        double nextGoalAngle = speed.normalizedValue().angleTo(target - location);

        double distanceMultiplier = std::clamp(pow(nextGoalDist, 2) / pow(distMultRange, 2),
                                               0.5,
                                               1.0);
        double nextAngleMultiplier = (Vector(goals[nextGoalId]) - location).len() < 3000 && strategy != Strategy::ATTACK ?
                std::clamp(getNextAngleMultiplier(),
                           0.25,
                           1.0)
                : 1.0;
        double enemyDistanceMultiplier = closestEnemyDistance() < enemyDistMultRange ? 2.55 : 1;
        double angleMultiplier = std::abs(angle) < angleMultThreshold ?
                std::clamp((1 - (nextGoalAngle / maxAbsAngle)),
                           0.1,
                           1.0)
                : 1.0;
#ifdef LOG
        std::cerr << "Multipliers: dist:" << distanceMultiplier;
        std::cerr << " angle: " << angleMultiplier;
        std::cerr << " enemy: " << enemyDistanceMultiplier;
        std::cerr << " nextAngle: " << nextAngleMultiplier << "\n";
#endif

        int thrust = static_cast<int>(maxThrust * distanceMultiplier * nextAngleMultiplier * enemyDistanceMultiplier * angleMultiplier);
        if(strategy == Strategy::ATTACK && aboutToCollideFriendly()) thrust = 0;

#ifdef LOG
        std::cerr << "\n";
#endif

        std::pair<int, int> targetPoint = target.toPoint();
        if (aboutToCollideEnemy() && calcSpeed.len() > minSpeedForShield)
        {
            return std::to_string(targetPoint.first) + " " + std::to_string(targetPoint.second) + " SHIELD";
        }
        else if (!boostUsed && (nextGoalDist >= minDisForBoost && nextGoalAngle < maxAngleForBoost))
        {
            boostUsed = true;
            return std::to_string(targetPoint.first) + " " + std::to_string(targetPoint.second) + " BOOST";
        }
        else
        {
            return std::to_string(targetPoint.first) + " "
                   + std::to_string(targetPoint.second) + " "
                   + std::to_string(std::clamp(thrust, 10, maxThrust));
        }
    }

    bool operator==(const Pod& other) const {
        return podId == other.podId;
    }

    [[nodiscard]] int getId() const {
        return podId;
    }

private:
    int podId;
    static int podCount;

    [[nodiscard]] double getNextAngleMultiplier() const {
        auto targetCheckpoint = Vector(goals[nextGoalId]);
        Vector fromNextCheckpoint = (targetCheckpoint
                                     - Vector(goals[(nextGoalId + 1) % goals.size()]))
                .normalizedValue();
        double angleDiff =  (calcSpeed.normalizedValue()).angleTo(fromNextCheckpoint);
        return angleDiff / maxAbsAngle;
    }

    static std::string strategyToString(const Strategy strategy) {
        switch (strategy) {
            case RACE: return "Race";
            case ATTACK: return "Attack";
            default: return "(Unknown Strategy)";
        }
    }

    [[nodiscard]] Vector getNextPointCorrectedTarget(const int range) const {
        if (progress.distanceToNext > calcSpeed.len()) {
            return nextPoint[nextGoalId] * range + goals[nextGoalId];
        } else {
            return (nextPoint[(nextGoalId + 1) % goals.size()]) *
                   range + goals[(nextGoalId + 1) % goals.size()];
        }

    }

    static Vector getBestEnemyTarget() {
        return bestEnemy->location + bestEnemy->calcSpeed * attackTargetLocationOffset;
    }

    Vector getTargetLocation() {
        switch (strategy) {
            case Pod::Strategy::RACE:
                return getNextPointCorrectedTarget(checkpointSize * 0.8);
            case Pod::Strategy::ATTACK:
                return getBestEnemyTarget();
        }
    }

    [[nodiscard]] Vector getSpeedCorrectedTarget(Vector target) const {
        Vector movementDirDiff = ((target - location).normalizedValue()
                - calcSpeed.normalizedValue()).normalizedValue();
        return (location + calcSpeed)
                + (target - (location + calcSpeed)
                + movementDirDiff * movementCorrectionWeight);

    }

    [[nodiscard]] double closestEnemyDistance() const {
        return std::min((enemyPods->first.location - location).len(),
                        (enemyPods->second.location - location).len());
    }

    [[nodiscard]] bool aboutToCollideEnemy() const {
        Vector nextLocation = location + speed;
        return std::min((nextLocation - enemyPods->first.location - enemyPods->first.speed).len(),
                        (nextLocation - enemyPods->second.location - enemyPods->second.speed).len()) < 850;

    }

    static bool aboutToCollideFriendly() {
        return (pods->second.location + pods->second.calcSpeed - pods->first.location - pods->first.calcSpeed).len() < 1000;
    }

};
int Pod::podCount = 0;

std::pair<Pod, Pod> getOrderedPods(std::pair<Pod, Pod> toOrder){
    Progress first = toOrder.first.progress;
    Progress second = toOrder.second.progress;
#ifdef LOG
    std::cerr << "First " << toOrder.first.toString() << "\n";
    std::cerr << "Second " << toOrder.second.toString() << "\n";
#endif

    if(second.lap > first.lap
        || second.lap == first.lap
            && second.goal > first.goal
        || second.lap == first.lap
            && second.goal == first.goal
            && second.distanceToNext < first.distanceToNext)
    {
        return std::make_pair(toOrder.second, toOrder.first);
    }
    return toOrder;
}

void updatePodOrder(){
    std::pair<Pod, Pod> ordered = getOrderedPods(*pods);
    if(ordered.first == pods->first)
    {
        pods->first.strategy = Pod::Strategy::RACE;
        pods->second.strategy = Pod::Strategy::ATTACK;
    }
    else
    {
        pods->first.strategy = Pod::Strategy::ATTACK;
        pods->second.strategy = Pod::Strategy::RACE;
    }
    ordered = getOrderedPods(*enemyPods);
    if(ordered.first == enemyPods->first)
    {
        bestEnemy = &(enemyPods->first);
    }
    else
    {
        bestEnemy = &(enemyPods->second);
    }
#ifdef LOG
    std::cerr << "Best enemy: " << bestEnemy->toString() << std::endl;
#endif
}

void collectPodData(Pod* pod){
    int x, y, vx, vy;
    std::cin >> x >> y >> vx >> vy;
    pod->calcSpeed = pod->location.len() != 0 ? Vector(Vector(x, y) - pod->location) : Vector(0,0);
    pod->location = Vector(x, y);
    pod->speed = Vector(vx, vy);
    std::cin >> pod->angle >> pod-> nextGoalId;
    pod->angle = degToRad(pod->angle);
    pod->updateProgress();
#ifdef LOG
    std::cerr << "Done updating " << pod->toString() << "\n";
    std::cerr << "Pod" << pod->getId()
    << " nextGoalId " << pod->nextGoalId
    << " lap " << pod->progress.lap
    << " progressGoal " << pod->progress.goal<<"\n";
#endif
}

void updatePods(){
    collectPodData(&(pods->first));
    collectPodData(&(pods->second));
    collectPodData(&(enemyPods->first));
    collectPodData(&(enemyPods->second));
    updatePodOrder();
#ifdef LOG
    std::cerr << pods->first.toString() << std::endl;
    std::cerr << pods->second.toString() << std::endl;
    std::cerr << enemyPods->first.toString() << std::endl;
    std::cerr << enemyPods->second.toString() << std::endl;
#endif
}

void calculateNextPointVectors(){
    for(int i=0; i < goals.size(); i++){
        int nextIndex = (int)((i + 1) % goals.size());
        Vector toNext(goals[i].getX(), goals[i].getY(),
                      goals[nextIndex].getX(), goals[nextIndex].getY());
        toNext.normalize();
        nextPoint.push_back(toNext);
    }
}

void collectInitializationData() {
    int checkpointCount;
    std::cin >> laps >> checkpointCount;
    int x, y;
    for(int i=0; i < checkpointCount && std::cin >> x >> y; i++){
        goals.emplace_back(x, y);
    }
    calculateNextPointVectors();

}

int main() {
    std::cerr << "START" << std::endl;
    collectInitializationData();
    std::cerr << "INIT DATA COLLECT COMPLETE" << std::endl;
    pods = std::make_unique<std::pair<Pod, Pod> >(Pod(), Pod());
    enemyPods = std::make_unique<std::pair<Pod, Pod> >(Pod(), Pod());
    std::cerr << "INIT COMPLETE" << std::endl;
    while(1) {
        updatePods();
#ifdef LOG
        std::cerr << "POD UPDATE COMPLETE" << "\n";
#endif
        std::cin.ignore();
        std::cout << pods->first.getAction() << std::endl;
        std::cout << pods->second.getAction() << std::endl;
    }
}
#pragma clang diagnostic pop