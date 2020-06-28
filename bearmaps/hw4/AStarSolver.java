package bearmaps.hw4;
import bearmaps.proj2ab.DoubleMapPQ;

import edu.princeton.cs.algs4.Stopwatch;

import java.util.*;


public class AStarSolver<Vertex> implements ShortestPathsSolver<Vertex> {
    private DoubleMapPQ<Vertex> fringe;
    private List<Vertex> orderedSolution;
    private Map<Vertex,Double> distTo;
    private  Map<Vertex, Vertex> edgeTo;
    private double timeSpent;
    private Stopwatch sw;
    private AStarGraph<Vertex> graph;
    private Vertex start,end;
    private int numSolutions;
    private Vertex smallest;
    private SolverOutcome solvable;



    public AStarSolver(AStarGraph<Vertex> input, Vertex start, Vertex end, double timeout) {
        sw = new Stopwatch();
        this.graph = input;
        this.start = start;
        this.end = end;
        this.fringe = new DoubleMapPQ<>();
        this.orderedSolution = new LinkedList<>();

        this.distTo = new HashMap<>();
        this.edgeTo = new HashMap<>();
        this.numSolutions = 0;

        fringe.add(this.start, 0);
        distTo.put(this.start, 0.0);
        smallest = fringe.getSmallest();

        while (fringe.size() != 0 && !fringe.getSmallest().equals(this.end) && timeSpent < timeout) {
            Vertex smallest = fringe.removeSmallest();
            numSolutions +=1;
            for (WeightedEdge<Vertex> neighbor : this.graph.neighbors(smallest)) {
                relax(neighbor, edgeTo, distTo, graph, fringe, end);

            }
            timeSpent = sw.elapsedTime();
        }
        if (timeSpent > timeout) {
            this.solvable = SolverOutcome.TIMEOUT;
        } else if (fringe.size() == 0 && !smallest.equals(end)) {
            this.solvable = SolverOutcome.UNSOLVABLE;
        }
        else {
            Vertex p = this.end;
            while (!p.equals(this.start)) {

                ((LinkedList<Vertex>) orderedSolution).addFirst(p);
                p = edgeTo.get(p);

            }
            ((LinkedList<Vertex>) orderedSolution).addFirst(p);
            solvable = SolverOutcome.SOLVED;


        }
    }
    private void relax(WeightedEdge e, Map<Vertex,Vertex> edgeTo, Map<Vertex,Double>distTo, AStarGraph<Vertex> graph,DoubleMapPQ fringe, Vertex goal) {
        Vertex p = (Vertex) e.from();
        Vertex q = (Vertex) e.to();
        double weight = e.weight();
        if (!distTo.containsKey(q) || distTo.get(p)+weight < distTo.get(q) ) {
            distTo.put(q,distTo.get(p) + weight);
            if (fringe.contains(q)) {
                fringe.changePriority(q, distTo.get(q) + graph.estimatedDistanceToGoal(q, goal));
            }
            else if(!fringe.contains(q)) {
                fringe.add(q, distTo.get(q) + graph.estimatedDistanceToGoal(q, goal));

            }
            edgeTo.put(q,p);
        }


    }
    public SolverOutcome outcome() {
        return solvable;

    }
    public List<Vertex> solution() {
        return orderedSolution;

    }
    public double solutionWeight() {
        return distTo.get(end);

    }
    public int numStatesExplored() {
        return numSolutions;


    }
    public double explorationTime() {
        return timeSpent;

    }
}