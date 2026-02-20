#include<bits/stdc++.h>

using namespace std;

// Enum for the three heuristics
enum Heuristic { MISPLACED, MANHATTAN, LINEAR_CONFLICT };

// Structure to represent a node in the A* search tree
struct Node {
    string state;
    int g; // Cost so far (depth)
    int h; // Heuristic cost to goal
    int f; // Total cost (g + h)

    // For priority queue to order by lowest f-cost, then highest g-cost
    bool operator>(const Node &other) const {
        if(f == other.f) return g < other.g;
        return f > other.f;
    }
};

// --- 1. Solvability Check ---
int countInversions(const string &state) {
    int inversions = 0;
    string linear = "";
    for(char c : state) {
        if(c != '0') linear += c;
    }

    for(int i = 0; i < linear.length() - 1; i++) {
        for(int j = i + 1; j < linear.length(); j++) {
            if(linear[i] > linear[j]) {
                inversions++;
            }
        }
    }
    return inversions;
}

bool isSolvable(const string &startState, const string &goalState) {
    int startInv = countInversions(startState);
    int goalInv = countInversions(goalState);
    return (startInv % 2) == (goalInv % 2);
}

// --- 2. Heuristic Functions ---

// a. Misplaced Tiles
int getMisplacedTiles(const string &state, const string &goalState) {
    int count = 0;
    for(int i = 0; i < 9; i++) {
        if(state[i] != '0' && state[i] != goalState[i]) {
            count++;
        }
    }
    return count;
}

// b. Manhattan Distance
int getManhattanDistance(const string &state, const string &goalState) {
    int distance = 0;
    for(int i = 0; i < 9; i++) {
        if(state[i] != '0') {
            int targetIndex = goalState.find(state[i]);
            int currRow = i / 3, currCol = i % 3;
            int targetRow = targetIndex / 3, targetCol = targetIndex % 3;
            distance += abs(currRow - targetRow) + abs(currCol - targetCol);
        }
    }
    return distance;
}

// c. Linear Conflict (Rows + Columns)
int getLinearConflict(const string &state, const string &goalState) {
    int distance = getManhattanDistance(state, goalState);
    int conflicts = 0;

    // Row Conflicts
    for(int row = 0; row < 3; row++) {
        for(int i = 0; i < 2; i++) {
            for(int j = i + 1; j < 3; j++) {
                int idx1 = row * 3 + i;
                int idx2 = row * 3 + j;
                if(state[idx1] != '0' && state[idx2] != '0') {
                    int target1 = goalState.find(state[idx1]);
                    int target2 = goalState.find(state[idx2]);
                    if(target1 / 3 == row && target2 / 3 == row && target1 > target2) {
                        conflicts++;
                    }
                }
            }
        }
    }

    // Column Conflicts
    for(int col = 0; col < 3; col++) {
        for(int i = 0; i < 2; i++) {
            for(int j = i + 1; j < 3; j++) {
                int idx1 = i * 3 + col;
                int idx2 = j * 3 + col;
                if(state[idx1] != '0' && state[idx2] != '0') {
                    int target1 = goalState.find(state[idx1]);
                    int target2 = goalState.find(state[idx2]);
                    if(target1 % 3 == col && target2 % 3 == col && target1 > target2) {
                        conflicts++;
                    }
                }
            }
        }
    }
    return distance + (2 * conflicts);
}

// Helper to route to the correct heuristic
int calculateHeuristic(const string &state, const string &goalState, Heuristic hType) {
    switch(hType) {
    case MISPLACED: return getMisplacedTiles(state, goalState);
    case MANHATTAN: return getManhattanDistance(state, goalState);
    case LINEAR_CONFLICT: return getLinearConflict(state, goalState);
    }
    return 0;
}

// --- 3. A* Algorithm Engine ---
void solveAStarToFile(const string &startState, const string &goalState, Heuristic hType, const string &hName, ofstream &outFile) {
    outFile << "======================================\n";
    outFile << "Heuristic: " << hName << "\n";
    outFile << "======================================\n";

    auto startTime = chrono::high_resolution_clock::now();

    priority_queue<Node, vector<Node>, greater<Node>> frontier;
    unordered_set<string> explored;
    unordered_map<string, string> parentMap;

    Node startNode = { startState, 0, calculateHeuristic(startState, goalState, hType), 0 };
    startNode.f = startNode.g + startNode.h;

    frontier.push(startNode);
    parentMap[startState] = "";

    int nodesRemoved = 0;
    int dr[] = { -1, 1, 0, 0 };
    int dc[] = { 0, 0, -1, 1 };

    while(!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();
        nodesRemoved++;

        // Goal Check
        if(current.state == goalState) {
            auto endTime = chrono::high_resolution_clock::now();
            chrono::duration<double> elapsedSeconds = endTime - startTime;

            outFile << "Status: Solution Found!\n";
            outFile << "- Running Time: " << elapsedSeconds.count() << " seconds\n";
            outFile << "- Solution Length (depth): " << current.g << "\n";
            outFile << "- Nodes Removed from Frontier: " << nodesRemoved << "\n\n";

            // We will only print the step-by-step path for the final heuristic (Linear Conflict)
            // to avoid cluttering the text file with 3 identical massive paths.
            if(hType == LINEAR_CONFLICT) {
                vector<string> path;
                string step = current.state;
                while(step != "") {
                    path.push_back(step);
                    step = parentMap[step];
                }
                reverse(path.begin(), path.end());

                outFile << "--- Step-by-Step Path (Linear Conflict) ---\n";
                for(int i = 0; i < path.size(); i++) {
                    outFile << "Step " << i << ":\n";
                    outFile << path[i].substr(0, 3) << "\n";
                    outFile << path[i].substr(3, 3) << "\n";
                    outFile << path[i].substr(6, 3) << "\n\n";
                }
            }
            return;
        }

        explored.insert(current.state);

        int zeroIdx = current.state.find('0');
        int r = zeroIdx / 3, c = zeroIdx % 3;

        for(int i = 0; i < 4; i++) {
            int newR = r + dr[i], newC = c + dc[i];

            if(newR >= 0 && newR < 3 && newC >= 0 && newC < 3) {
                int newZeroIdx = newR * 3 + newC;
                string neighborState = current.state;
                swap(neighborState[zeroIdx], neighborState[newZeroIdx]);

                if(explored.find(neighborState) == explored.end()) {
                    int hCost = calculateHeuristic(neighborState, goalState, hType);
                    Node neighborNode = { neighborState, current.g + 1, hCost, current.g + 1 + hCost };

                    if(parentMap.find(neighborState) == parentMap.end() || current.g + 1 < neighborNode.g) {
                        parentMap[neighborState] = current.state;
                        frontier.push(neighborNode);
                    }
                }
            }
        }
    }
    outFile << "Failed to find a solution.\n\n";
}

int main() {
    string startState, goalState;

    cout << "=== 8-Puzzle Multi-Heuristic Evaluator ===" << endl;
    cout << "Enter Start State (9 digits, use '0' for blank): ";
    cin >> startState;

    cout << "Enter Goal State (9 digits, use '0' for blank): ";
    cin >> goalState;

    if(!isSolvable(startState, goalState)) {
        cout << "\nError: The given start state cannot reach the provided goal state.\n";
        cout << "(Their inversion parities do not match). Please try again.\n";
        return 1;
    }

    string filename = "output.txt";
    ofstream outFile(filename);

    if(!outFile.is_open()) {
        cerr << "Error: Could not open file " << filename << " for writing." << endl;
        return 1;
    }

    outFile << "8-Puzzle Experimentation Results\n";
    outFile << "Start State: " << startState << "\n";
    outFile << "Goal State:  " << goalState << "\n\n";

    solveAStarToFile(startState, goalState, MISPLACED, "Misplaced Tiles", outFile);

    solveAStarToFile(startState, goalState, MANHATTAN, "Manhattan Distance", outFile);

    solveAStarToFile(startState, goalState, LINEAR_CONFLICT, "Linear Conflict", outFile);

    outFile.close();
    cout << "\nSuccess! Check the output file '" << filename << "'." << endl;

    return 0;
}