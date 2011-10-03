using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Hamlet.Algorithms.OptimalSpanningSubgraphs
{
    /// <summary>
    /// Represents an implementation af an algorithm to solve the connector problem.
    /// </summary>
    public interface IConnectorProvider<V, E>
        where V : IEquatable<V>
        where E : IEquatable<E>
    {
        /// <summary>
        /// Returns the optimal spanning tree of the wrapped graph, i.e. a spanning tree whose wheight is
        /// the minimum possible weight.
        /// </summary>
        ColoredTree<V, E> Compute();
    }

    /// <summary>
    /// Implements the Kruskal algorithm to find the optimal spanning subtree.
    /// </summary>
    public class KruskalAlgorithmProvider
    {
        public static KruskalAlgorithmProvider<V, E> Create<V, E>(WeightedGraph<V, E> graph)
            where V : IEquatable<V>
            where E : IEquatable<E>
        {
            return new KruskalAlgorithmProvider<V, E>(graph);
        }
    }

    /// <summary>
    /// Implements the Kruskal algorithm to find the optimal spanning subtree.
    /// </summary>
    public class KruskalAlgorithmProvider<V, E> : KruskalAlgorithmProvider, IConnectorProvider<V, E>
        where V : IEquatable<V>
        where E : IEquatable<E>
    {
        protected WeightedGraph<V, E> graph;
        private Dictionary<WeightedGraph<V, E>.Vertex, WeightedGraph<V, E>.Vertex> parent;

        public KruskalAlgorithmProvider(WeightedGraph<V, E> graph)
        {
            this.graph = graph;
            parent = new Dictionary<ColoredGraph<V, E>.Vertex, ColoredGraph<V, E>.Vertex>();
        }

        public ColoredTree<V, E> Compute()
        {
            if (!graph.IsDirected || !graph.IsConnected())
                throw new InvalidOperationException("Kruskal algorithm only works for connected digraphs.");

            this.InitializeData();

            var orderedEdges = graph.Edges.OrderBy(graph.EdgeWeight);
            var orderedEdgesEnumerator = orderedEdges.GetEnumerator();
            List<WeightedGraph<V, E>.Edge> addedEdges = new List<ColoredGraph<V, E>.Edge>();

            while (addedEdges.Count < graph.Order - 1)
            {
                if (!orderedEdgesEnumerator.MoveNext())
                    throw new GraphException("A minimal spanning tree for this graph does not exists.");

                var newEdge = orderedEdgesEnumerator.Current;
                var startRoot = this.GetRoot(newEdge.Start);
                var endRoot = this.GetRoot(newEdge.End);
                if (!startRoot.Equals(endRoot))
                {
                    if (graph.FindEdges(parent[startRoot], endRoot).Count() > 0)
                        parent[startRoot] = endRoot;
                    else
                        parent[endRoot] = startRoot;
                    addedEdges.Add(newEdge);
                }
            }

            WeightedGraph<V, E> result = new WeightedGraph<V, E>(graph.Weight);

            foreach (var vertex in graph.Vertices)
                result.AddVertex(vertex.Data);

            foreach (var edge in addedEdges)
                result.AddEdge(edge.Start.Index, edge.End.Index, edge.Data);

            this.DisposeData();

            return result.ForceTree();
        }

        private void InitializeData()
        {
            foreach (var vertex in graph.Vertices)
                parent.Add(vertex, vertex);
        }

        private void DisposeData()
        {
            parent.Clear();
        }

        private WeightedGraph<V, E>.Vertex GetRoot(WeightedGraph<V, E>.Vertex vertex)
        {
            var tmpParent = vertex;
            while (tmpParent != parent[tmpParent])
                tmpParent = parent[tmpParent];
            return tmpParent;
        }
    }
}
