 # Assignment 3
 # SECTION A – Theory Questions (Short Answers)
 
  1. Explain polynomial time reducibility with example.
Ans. Polynomial time reducibility is a concept from computational complexity theory that describes how one problem can be efficiently transformed or mapped into another problem so that solving the second problem also          gives a solution to the first problem. If this transformation (called a reduction) can be performed in polynomial time, it means the first problem is "no harder" than the second problem in terms of computational          effort.
    Example: 3-SAT to Subset Sum
    A classic example is the reduction from the 3-SAT problem to the Subset Sum problem:

     3-SAT: Given a Boolean formula in 3-CNF (each clause has three literals), is there an assignment of variables that makes the formula true?

     Subset Sum: Given a set of integers, is there a subset whose sum equals a target value?

     To prove Subset Sum is NP-complete, one can show a polynomial-time reduction from 3-SAT to Subset Sum: Any instance of 3-SAT can be transformed (using a systematic mapping) into an instance of Subset Sum in               polynomial time, such that solving the Subset Sum instance answers the original 3-SAT question. This shows Subset Sum is at least as hard as 3-SAT; if Subset Sum can be solved efficiently, so can 3-SAT.




 
