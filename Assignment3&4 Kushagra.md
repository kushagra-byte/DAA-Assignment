 # Assignment 3
 # SECTION A – Theory Questions (Short Answers)
 
  1. Explain polynomial time reducibility with example.

Ans. Polynomial time reducibility is a concept from computational complexity theory that describes how one problem can be efficiently transformed or mapped into another problem so that solving the second problem also          gives a solution to the first problem. If this transformation (called a reduction) can be performed in polynomial time, it means the first problem is "no harder" than the second problem in terms of computational          effort.
    Example: 3-SAT to Subset Sum
    A classic example is the reduction from the 3-SAT problem to the Subset Sum problem:

     3-SAT: Given a Boolean formula in 3-CNF (each clause has three literals), is there an assignment of variables that makes the formula true?

     Subset Sum: Given a set of integers, is there a subset whose sum equals a target value?

     To prove Subset Sum is NP-complete, one can show a polynomial-time reduction from 3-SAT to Subset Sum: Any instance of 3-SAT can be transformed (using a systematic mapping) into an instance of Subset Sum in polynomial time, such that solving the Subset Sum instance answers the original 3-SAT question. This shows Subset Sum is at least as hard as 3-SAT; if Subset Sum can be solved efficiently, so can 3-SAT.

2. Define and differentiate:

   NP Packet Graph

   NP Scheduling Problem

Ans. The NP Packet Graph problem and NP Scheduling Problem are both problems in computational complexity theory related to finding efficient solutions, but they focus on different domains: graphs and scheduling respectively.


NP Packet Graph

    *The NP Packet Graph problem refers to computational challenges that involve routing or scheduling packets through a network represented as a graph.​
    *These problems typically revolve around efficiently determining paths or scheduling transmissions so packets can move from their sources to destinations within constraints such as minimal makespan, avoiding congestion, or minimizing delays.​
    *Many packet routing problems are NP-hard, meaning no polynomial-time solution is known; verifying a proposed solution (such as a schedule or routing) can be done in polynomial time, which places them in the NP class.​
    Example: Given a directed graph, a set of packets with predefined paths, and a time horizon, determine the shortest schedule for routing all packets without collision, a problem known to be NP-hard even in restricted network topologies.​

NP Scheduling Problem

    *The NP Scheduling Problem covers a family of decision problems focused on efficiently assigning tasks, jobs, or processes to resources (such as processors, workers, or machines).​
    *Types of NP scheduling problems include job-shop scheduling, flow-shop scheduling, and task assignment, with constraints like deadlines, task durations, and resource availability.
    *Typically, given tasks and constraints, one must decide if a feasible schedule exists that meets all requirements, such as finishing within a certain time or using no more than a given number of resources.​
    *These problems are NP because, if given a proposed schedule, it can be checked efficiently (in polynomial time) whether it meets the constraints.
    
    
    
    
3.What is a non-deterministic algorithm? Give one example

Ans. A non-deterministic algorithm is one that can exhibit different behaviors or follow multiple potential paths for the same input in different runs. Unlike deterministic algorithms, which produce a fixed output for a given input, non-deterministic algorithms can "guess" from several choices at certain steps and explore many possible computation paths simultaneously. The algorithm is considered successful if at least one of these paths produces the correct solution.


Example: Non-deterministic Search Algorithm

     Consider the problem of searching an element x in an array A[1:n]:
     * A non-deterministic algorithm for this problem can "guess" an index j where x might be present.
     * It then verifies whether A[j[ = x 
     * If yes , it returns j ; otherwise it fails.
     * This guess-and-check mechanism models how a non-deterministic algorithm works, verifying correctness in polynomial time if a guess is right.

4. What is a state space tree? State two ways to search for an answer node.

Ans. A state space tree is a tree representation of all possible states (or configurations) of a problem starting from an initial state (root) and branching out to represent all possible successor states until the goal state is reached or all states have been explored. It is a conceptual tool used to explore the solutions of a problem by systematically examining all states and their transitions.

Two ways to search for an answer node in a state space tree:

Depth-First Search (DFS): This method explores as far as possible along each branch before backtracking. It goes deep down a single path until it reaches a goal state or a dead end, then backtracks to explore other branches.

Breadth-First Search (BFS): This approach explores all nodes at the current level before moving to the next level. It systematically explores neighbor states first, ensuring that the shortest path to a goal is found if one exists


5. Explain Boolean Satisfiability (SAT) problem with one example.


5.Explain Boolean Satisfiability (SAT) problem with one example.

Ans.  The Boolean Satisfiability Problem (SAT) is a fundamental problem in logic and computer science that asks whether there exists an assignment of truth values (TRUE or FALSE) to variables in a given Boolean formula that makes the entire formula evaluate to TRUE. If such an assignment exists, the formula is said to be satisfiable; if not, it is unsatisfiable.

Consider the formula F= a^ -b (a AND NOT b ): 
              
     * This formula is satisfied because you can assign a = TRUE and b = FALSE, making F evaluates to TRUE 
     * Conversely , the formula G=a^-a is unsatisfied because no assignment to a can both a and its negation TRUE simulataneously. 


6. Define Cook’s Theorem in one line and state one application.

Ans  Cook's Theorem states that the Boolean Satisfiability Problem (SAT) is NP-complete, meaning every problem in NP can be reduced to SAT in polynomial time 

Application: It serves as the foundation for proving other problems NP-complete by polynomial-time reductions from SAT, such as 3-SAT, Clique, and Subset Sum problems


7. Differentiate:

P, NP, NP-complete, NP-hard

Write two examples for each class (short titles only).


Ans.    P (Polynomial time):

     *Problems solvable in polynomial time by a deterministic algorithm
     *Solutions found efficiently.
     *Examples: Sorting, Graph Connectivity.

   NP (Nondeterministic Polynomial time):
     
     * Solutions verifiable in polynomial time by a deterministic algorithm.
     May or may not be solvable efficiently
     Examples: Hamiltonian Path, Composite Number.

  NP-complete

    *Problems in NP that are as hard as any problem in NP (NP-hard).
    *If one NP-complete problem can be solved in polynomial time, all NP problems can.
    *Examples: Boolean Satisfiability (SAT), Vertex Cover.

 NP-hard
    
    *Problems at least as hard as NP-complete problems.
    *Not necessarily in NP or even decision problems.
    *Examples: Halting Problem, Traveling Salesman Optimization.




8. Explain the Hamiltonian Cycle problem in context of NP.

Ans. The Hamiltonian Cycle problem, in the context of NP, is described as follows:

    *It asks whether a given graph contains a cycle that visits each vertex exactly once and returns to the starting vertex.
    *It belongs to the class NP because a proposed cycle (certificate) can be verified in polynomial time by checking the sequence of vertices and edges.
    *The problem is NP-complete since it is both in NP and NP-hard.
    *NP-hardness is shown by polynomial-time reductions from known NP-complete problems (e.g., Vertex Cover, Hamiltonian Path).
    *Finding a Hamiltonian Cycle is computationally difficult, but verifying a given cycle is efficient.
    *Due to its NP-completeness, solving Hamiltonian Cycle efficiently would imply efficient solutions for all problems in NP.


# SECTION B – Algorithm Applications & Coding

9. Dijkstra’s Algorithm

State the core logic in one line.

Write pseudocode or main function steps only.


Ans.  Dijkstra’s algorithm finds the shortest path from a source node to all other nodes in a weighted graph by iteratively selecting the node with the smallest tentative distance and relaxing its neighbors.

    function Dijkstra(Graph, source):
       for each vertex v in Graph:
           dist[v] = infinity
           previous[v] = undefined
       dist[source] = 0
       Q = set of all vertices in Graph
     
    while Q is not empty:
        u = vertex in Q with smallest dist[u]
        remove u from Q
        
        for each neighbor v of u:
            alt = dist[u] + weight(u, v)
            if alt < dist[v]:
                dist[v] = alt
                previous[v] = u
                
    return dist, previous


10.  N-Queen Problem

Write only the key recursive function.

Add minimal comments for clarity.

Ans.  

    bool solveNQueen(int col, int n, int board[][N]) {
        if (col >= n)  // All queens placed successfully
            return true;
    
    for (int row = 0; row < n; row++) {
        if (isSafe(row, col, board, n)) {  // Check if queen can be placed
            board[row][col] = 1;  // Place queen
            
            if (solveNQueen(col + 1, n, board))  // Recur to place rest
                return true;
            
            board[row][col] = 0;  // Backtrack: remove queen
        }
    }
    return false;  // No placement possible in this column
    }


   This function tries to place queens column by column using recursion and backtracking.

    isSafe() checks that no queen attacks the position.

    If placement fails, it backtracks to try other positions.


11. Dynamic Programming – 0/1 Knapsack Problem

Weights: {3, 4, 6, 5}

Profits: {2, 3, 1, 4}

Capacity: 8

Write a DP function/pseudocode and print total profit.


Ans.

    def knapsack(profits, weights, capacity):
        n = len(profits)
        dp = [[0 for _ in range(capacity + 1)] for _ in range(n + 1)]

    for i in range(1, n + 1):
        for w in range(1, capacity + 1):
            if weights[i-1] <= w:
                dp[i][w] = max(profits[i-1] + dp[i-1][w - weights[i-1]],
                               dp[i-1][w])
            else:
                dp[i][w] = dp[i-1][w]

    return dp[n][capacity]

    # Given data
    weights = [3, 4, 6, 5]
    profits = [2, 3, 1, 4]
    capacity = 8

    # Calculate and print total profit
    total_profit = knapsack(profits, weights, capacity)
    print("Total Profit:", total_profit)


Explanation:

     dp[i][w] stores the max profit considering the first i items with capacity w.

    Either include the current item (if fits) or exclude it for max profit.

    The answer is in dp[n][capacity].


12.  Travelling Salesman Problem – Branch & Bound

Take nodes A, B, C, D with edge weights (example: AB=10, AC=6, AD=8, BC=5, BD=9, CD=7).

State essential pseudocode steps to solve this problem.

Ans 

Here are the essential pseudocode steps for solving the Travelling Salesman Problem (TSP) using Branch and Bound with nodes A, B, C, D and given edge weights:

    1. Initialize:
    - Current best solution cost as infinity
    - Priority queue for live nodes (partial paths with lower bound)

    2. Start from the root node representing the starting city (e.g., A)
    - Calculate lower bound for initial node (using edge weights)
    - Add root to the priority queue

    3. While priority queue is not empty:
    a. Extract node with the smallest lower bound (best promising node)
    
    b. If node represents a complete tour (all cities visited):
        - Update best solution if current cost < best cost
        
    c. Else, expand node:
        - For each unvisited city:
            i. Generate child node by adding city to path
            ii. Compute new cost and lower bound for child node
            iii. If lower bound < current best cost, add child to queue (prune others)

    4. Return best solution and cost after queue is empty

Explaination 

    Lower bound estimation is crucial for pruning ineffective paths early.

    The search is guided by best lower bound, ensuring efficiency.

    This approach systematically explores partial tours while pruning suboptimal solutions.



13. Floyd-Warshall Algorithm

Explain core idea in one short sentence.

Write the main function or pseudocode for generating the shortest path matrix.

 
Ans.  Floyd-Warshall algorithm computes shortest paths between all pairs of vertices by incrementally considering each vertex as an intermediate point to improve path distances.



    def floydWarshall(graph):
        n = len(graph)
        dist = [row[:] for row in graph]  # Clone graph adjacency matrix
    
    for k in range(n):
        for i in range(n):
            for j in range(n):
                if dist[i][k] + dist[k][j] < dist[i][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
    
    return dist

graph is a 2D matrix of edge weights (infinity if no direct edge).

dist[i][j] holds shortest distance from vertex i to j after considering intermediate vertices up to k.

The algorithm runs in  O(n^3) time for for a graph with n vertices .
    
