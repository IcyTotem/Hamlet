using System;
using System.Collections.Generic;
using System.Linq;

namespace Hamlet.Algorithms.ShortestPaths
{
    using Hamlet.Extensions;

    /// <summary>
    /// Represents an implementation of a single-source shortest paths research algorithm.
    /// </summary>
    public interface ISingleSourceShortestPathsProvider<V, E>
        where V : IEquatable<V>
        where E : IEquatable<E>
    {
        /// <summary>
        /// Returns an array of paths starting from source and going to each other vertex in the graph.
        /// Each path is the shortest possible path between its two ends.
        /// </summary>
        WeightedGraph<V, E>.Walk[] Compute(WeightedGraph<V, E>.Vertex source);
    }

    /// <summary>
    /// Represents an implementation of an all-pairs shortest paths research algorithm.
    /// </summary>
    public interface IAllPairsShortestPathsProvider<V, E>
        where V : IEquatable<V>
        where E : IEquatable<E>
    {
        /// <summary>
        /// Returns a matrix of paths in which the (i, j) item contains the shortest path starting from
        /// the i-th vertex and ending at the j-th vertex.
        /// </summary>
        WeightedGraph<V, E>.Walk[,] Compute();
    }

    /// <summary>
    /// Implements the Dijkstra algorithm for single-source shortest paths research on directed weighted graphs with nonnegative weights.
    /// </summary>
    public class DijkstraAlgorithmProvider
    {
        public static DijkstraAlgorithmProvider<V, E> Create<V, E>(WeightedGraph<V, E> graph)
            where V : IEquatable<V>
            where E : IEquatable<E>
        {
            return new DijkstraAlgorithmProvider<V, E>(graph);
        }
    }

    /// <summary>
    /// Implements the Dijkstra algorithm for single-source shortest paths research on directed weighted graphs with nonnegative weights.
    /// </summary>
    public class DijkstraAlgorithmProvider<V, E> : DijkstraAlgorithmProvider, ISingleSourceShortestPathsProvider<V, E>
        where V : IEquatable<V>
        where E : IEquatable<E>
    {
        private class VertexData
        {
            public Double Distance { get; set; }
            public WeightedGraph<V, E>.Edge IncomingEdge { get; set; }
            public Boolean Removed { get; set; }
        }

        private Dictionary<WeightedGraph<V, E>.Vertex, VertexData> data;
        protected WeightedGraph<V, E> graph;

        public DijkstraAlgorithmProvider(WeightedGraph<V, E> graph)
        {
            this.graph = graph;
            data = new Dictionary<ColoredGraph<V, E>.Vertex, VertexData>();
        }

        public WeightedGraph<V, E>.Walk[] Compute(ColoredGraph<V, E>.Vertex source)
        {
            if (!graph.IsDirected || graph.Edges.Any(e => graph.EdgeWeight(e) < 0))
                throw new InvalidOperationException("Dijkstra algorithm only works for directed graphs with nonnegative weights.");

            this.InitializeData(source);
            this.RelaxEdges();

            var result = this.CreateCompletePaths();
            this.DisposeData();

            return result;
        }

        private void InitializeData(WeightedGraph<V, E>.Vertex source)
        {
            foreach (var v in graph.Vertices)
                data.Add(v, new VertexData() { Distance = Double.PositiveInfinity, IncomingEdge = null, Removed = false });
            data[source].Distance = 0;
        }

        private void DisposeData()
        {
            data.Clear();
        }

        private void RelaxEdges()
        {
            while (!data.Values.All(vertexData => vertexData.Removed))
            {
                // Find the minimum distance among the unvisited vertices
                Double minDistance = data.Values.Min(vertexData => vertexData.Removed ? Double.PositiveInfinity : vertexData.Distance);

                if (Double.IsPositiveInfinity(minDistance))
                    break;

                // Find the nearest vertex among the remaining vertices
                var u = data.First(pair => pair.Value.Distance == minDistance && !pair.Value.Removed).Key;
                data[u].Removed = true;

                // Find the edges that start from u (and thus all of u's neighbours)
                var departingEdges = graph.FindEdges(u, EdgeEnds.Start).Where(e => !data[e.End].Removed);

                foreach (var e in departingEdges)
                {
                    var v = e.End;
                    Double alt = data[u].Distance + graph.EdgeWeight(e);

                    // Relaxation
                    if (alt < data[v].Distance)
                    {
                        data[v].Distance = alt;
                        data[v].IncomingEdge = e;
                        data[v].Removed = false;
                    }
                }
            }
        }

        /// <summary>
        /// Conversion from raw result to strongly typed data. Creates the complete path
        /// from source to v (for each v) attaching each edge to its predecessor
        /// </summary>
        private WeightedGraph<V, E>.Walk[] CreateCompletePaths()
        {
            WeightedGraph<V, E>.Walk[] result = new WeightedGraph<V, E>.Walk[graph.Order];
            foreach (var v in graph.Vertices)
            {
                List<WeightedGraph<V, E>.Edge> walk = new List<ColoredGraph<V, E>.Edge>();
                var current = v;
                while (data[current].IncomingEdge != null)
                {
                    walk.Insert(0, data[current].IncomingEdge);
                    current = data[current].IncomingEdge.Start;
                }
                result[v.Index] = new WeightedGraph<V, E>.Walk(walk);
            }
            return result;
        }
    }

    /// <summary>
    /// Implements the Bellman-Ford algorithm for single-source shortest paths research on directed weighted graphs.
    /// </summary>
    public class BellmanFordAlgorithmProvider
    {
        public static BellmanFordAlgorithmProvider<V, E> Create<V, E>(WeightedGraph<V, E> graph)
            where V : IEquatable<V>
            where E : IEquatable<E>
        {
            return new BellmanFordAlgorithmProvider<V, E>(graph);
        }
    }

    /// <summary>
    /// Implements the Bellman-Ford algorithm for single-source shortest paths research on directed weighted graphs.
    /// </summary>
    public class BellmanFordAlgorithmProvider<V, E> : BellmanFordAlgorithmProvider, ISingleSourceShortestPathsProvider<V, E>
        where V : IEquatable<V>
        where E : IEquatable<E>
    {
        private class VertexData
        {
            public Double Distance { get; set; }
            public WeightedGraph<V, E>.Edge IncomingEdge { get; set; }
        }

        private Dictionary<WeightedGraph<V, E>.Vertex, VertexData> data;
        protected WeightedGraph<V, E> graph;

        public BellmanFordAlgorithmProvider(WeightedGraph<V, E> graph)
        {
            this.graph = graph;
            data = new Dictionary<ColoredGraph<V, E>.Vertex, VertexData>();
        }

        public WeightedGraph<V, E>.Walk[] Compute(ColoredGraph<V, E>.Vertex source)
        {
            if (!graph.IsDirected)
                throw new InvalidOperationException("Bellman-Ford algorithm only works for directed graphs.");

            this.InitializeData(source);
            this.RelaxEdges();
            var result = this.CreateCompletePaths();
            this.DisposeData();

            return result;
        }

        private void InitializeData(WeightedGraph<V, E>.Vertex source)
        {
            foreach (var v in graph.Vertices)
                data.Add(v, new VertexData() { Distance = Double.PositiveInfinity, IncomingEdge = null });
            data[source].Distance = 0;
        }

        private void DisposeData()
        {
            data.Clear();
        }

        private void RelaxEdges()
        {
            for (Int32 i = 1; i < graph.Order; i++)
                foreach (var e in graph.Edges)
                {
                    // Relaxation
                    if (data[e.Start].Distance + graph.EdgeWeight(e) < data[e.End].Distance)
                    {
                        data[e.End].Distance = data[e.Start].Distance + graph.EdgeWeight(e);
                        data[e.End].IncomingEdge = e;
                    }
                }
        }

        private WeightedGraph<V, E>.Walk[] CreateCompletePaths()
        {
            WeightedGraph<V, E>.Walk[] result = new WeightedGraph<V, E>.Walk[graph.Order];
            foreach (var v in graph.Vertices)
            {
                List<WeightedGraph<V, E>.Edge> walk = new List<ColoredGraph<V, E>.Edge>();
                var current = v;
                while (data[current].IncomingEdge != null)
                {
                    walk.Insert(0, data[current].IncomingEdge);
                    current = data[current].IncomingEdge.Start;
                }
                result[v.Index] = new WeightedGraph<V, E>.Walk(walk);
            }
            return result;
        }
    }

    /// <summary>
    /// Implements the Floyd-Warshall algorithm for all-pairs shortest paths research on directed weighted graphs.
    /// </summary>
    public class FloydWarshallAlgorithmProvider
    {
        public static FloydWarshallAlgorithmProvider<V, E> Create<V, E>(WeightedGraph<V, E> graph)
            where V : IEquatable<V>
            where E : IEquatable<E>
        {
            return new FloydWarshallAlgorithmProvider<V, E>(graph);
        }
    }

    /// <summary>
    /// Implements the Floyd-Warshall algorithm for all-pairs shortest paths research on directed weighted graphs.
    /// </summary>
    public class FloydWarshallAlgorithmProvider<V, E> : FloydWarshallAlgorithmProvider, IAllPairsShortestPathsProvider<V, E>
        where V : IEquatable<V>
        where E : IEquatable<E>
    {
        private class FloydWarshallVertexData
        {
            public Double Distance { get; set; }
            public Int32 IntermediateVertexIndex { get; set; }
        }

        private FloydWarshallVertexData[,] data;
        protected WeightedGraph<V, E> graph;

        public FloydWarshallAlgorithmProvider(WeightedGraph<V, E> graph)
        {
            this.graph = graph;
        }

        public WeightedGraph<V, E>.Walk[,] Compute()
        {
            if (!graph.IsDirected)
                throw new InvalidOperationException("Floyd-Warshall algorithm only works for directed graphs.");

            this.InitializeData();
            this.RelaxEdges();
            var result = this.CreateCompletePaths();
            this.DisposeData();

            return result;
        }

        private void InitializeData()
        {
            data = new FloydWarshallVertexData[graph.Order, graph.Order];
            for (Int32 i = 0; i < graph.Order; i++)
                for (Int32 j = 0; j < graph.Order; j++)
                    data[i, j] = new FloydWarshallVertexData() { Distance = (i == j ? 0 : Double.PositiveInfinity), IntermediateVertexIndex = -1 };
            
            foreach (var e in graph.Edges)
                data[e.Start.Index, e.End.Index].Distance = graph.EdgeWeight(e);
        }

        private void DisposeData()
        {
            data = null;
        }

        private void RelaxEdges()
        {
            for (Int32 k = 0; k < graph.Order; k++)
                for (Int32 i = 0; i < graph.Order; i++)
                    for (Int32 j = 0; j < graph.Order; j++)
                        if (data[i, k].Distance + data[k, j].Distance < data[i, j].Distance)
                        {
                            data[i, j].Distance = data[i, k].Distance + data[k, j].Distance;
                            data[i, j].IntermediateVertexIndex = k;
                        }
        }

        /// <summary>
        /// Conversion from raw result to strongly typed data. Creates the complete path
        /// from each vertex to all the other vertices iteratively interpolating the set
        /// of vertex indices.
        /// </summary>
        private WeightedGraph<V, E>.Walk[,] CreateCompletePaths()
        {
            WeightedGraph<V, E>.Walk[,] result = new WeightedGraph<V, E>.Walk[graph.Order, graph.Order];
            for (Int32 i = 0; i < graph.Order; i++)
                for (Int32 j = 0; j < graph.Order; j++)
                {
                    if (Double.IsPositiveInfinity(data[i, j].Distance))
                        continue;

                    Int32[] indices = this.GetPathIndices(data, i, j).ToArray();
                    List<WeightedGraph<V, E>.Edge> walkEdges = new List<ColoredGraph<V, E>.Edge>();
                    for (Int32 h = 0; h < indices.Length - 1; h++)
                        walkEdges.AddRange(graph.FindEdges(graph.Vertices[indices[h]], graph.Vertices[indices[h + 1]]));

                    result[i, j] = new WeightedGraph<V, E>.Walk(walkEdges);
                }

            return result;
        }

        /// <summary>
        /// Interpolates the path between the i-th and j-th vertices in the graph, being data[i, j].IntermediateVertexIndex
        /// the index of the vertex you must pass through to go from i to j.
        /// </summary>
        private IEnumerable<Int32> GetPathIndices(FloydWarshallVertexData[,] data, Int32 i, Int32 j)
        {
            if (Double.IsPositiveInfinity(data[i, j].Distance))
                return new Int32[] { };

            Int32 intermediate = data[i, j].IntermediateVertexIndex;
            if (intermediate > -1)
                return GetPathIndices(data, i, intermediate).Concat(new Int32[] { intermediate }).Concat(GetPathIndices(data, intermediate, j));

            return new Int32[] { i, j };
        }
    }
}
