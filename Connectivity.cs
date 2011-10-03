using System;
using System.Collections.Generic;
using System.Linq;

namespace Hamlet.Algorithms.Connectivity
{
    /// <summary>
    /// Represents an implementation of a strongly connected components detection algorithm.
    /// </summary>
    public interface IStronglyConnectedComponentsProvider<V, E>
        where V : IEquatable<V>
        where E : IEquatable<E>
    {
        /// <summary>
        /// Returns a collection of all the strongly connected components in the graph.
        /// </summary>
        IEnumerable<ColoredGraph<V, E>.StronglyConnectedComponent> Compute();
    }

    /// <summary>
    /// Represents an implementation of a connectec components detection algorithm.
    /// </summary>
    public interface IConnectedComponentsProvider<V, E>
        where V : IEquatable<V>
        where E : IEquatable<E>
    {
        /// <summary>
        /// Returns a collection of all the connected components in the graph.
        /// </summary>
        IEnumerable<ColoredGraph<V, E>.ConnectedComponent> Compute();
    }
    
    /// <summary>
    /// Implements the Tarjan algorithm for strongly connected components detection on directed graphs.
    /// </summary>
    public class TarjanAlgorithmProvider
    {
        public static TarjanAlgorithmProvider<V, E> Create<V, E>(ColoredGraph<V, E> graph)
            where V : IEquatable<V>
            where E : IEquatable<E>
        {
            return new TarjanAlgorithmProvider<V, E>(graph);
        }

        public static TarjanAlgorithmProvider<V> Create<V>(Graph<V> graph)
            where V : IEquatable<V>
        {
            return new TarjanAlgorithmProvider<V>(graph);
        }
    }

    /// <summary>
    /// Implements the Tarjan algorithm for strongly connected components detection on directed graphs.
    /// </summary>
    public class TarjanAlgorithmProvider<V, E> : TarjanAlgorithmProvider, IStronglyConnectedComponentsProvider<V, E>
        where V : IEquatable<V>
        where E : IEquatable<E>
    {
        private class VertexData
        {
            public Int32 Index { get; set; }
            public Int32 LowLink { get; set; }
        }

        private Dictionary<ColoredGraph<V, E>.Vertex, VertexData> data;
        private Int32 index;
        private Stack<ColoredGraph<V, E>.Vertex> stack;
        private List<ColoredGraph<V, E>.StronglyConnectedComponent> results;
        protected ColoredGraph<V, E> graph;

        public TarjanAlgorithmProvider(ColoredGraph<V, E> graph)
        {
            this.graph = graph;
            data = new Dictionary<ColoredGraph<V, E>.Vertex, VertexData>();
            stack = new Stack<ColoredGraph<V, E>.Vertex>();
            results = new List<ColoredGraph<V, E>.StronglyConnectedComponent>();
        }

        public IEnumerable<ColoredGraph<V, E>.StronglyConnectedComponent> Compute()
        {
            if (!graph.IsDirected)
                throw new InvalidOperationException("Tarjan's algorithm only work for directed graphs.");

            this.InitializeData();

            foreach (var v in graph.Vertices)
                if (data[v].Index < 0)
                    this.StrongConnect(v);

            this.DisposeData();

            return results;
        }

        private void InitializeData()
        {
            index = 0;
            results.Clear();
            foreach (var v in graph.Vertices)
                data.Add(v, new VertexData() { Index = -1, LowLink = -1 });
        }

        private void DisposeData()
        {
            data.Clear();
            stack.Clear();
        }

        private void StrongConnect(ColoredGraph<V, E>.Vertex v)
        {
            data[v].Index = index;
            data[v].LowLink = index;
            index++;
            stack.Push(v);

            foreach (var vw in graph.FindEdges(v, EdgeEnds.Start))
            {
                var w = vw.End;
                if (data[w].Index < 0)
                {
                    this.StrongConnect(w);
                    data[v].LowLink = Math.Min(data[v].LowLink, data[w].LowLink);
                }
                else if (stack.Contains(w))
                    data[v].LowLink = Math.Min(data[v].LowLink, data[w].Index);
            }

            if (data[v].LowLink == data[v].Index)
            {
                // A new strongly connected component
                List<ColoredGraph<V, E>.Vertex> scc = new List<ColoredGraph<V, E>.Vertex>();
                ColoredGraph<V, E>.Vertex w;
                do
                {
                    w = stack.Pop();
                    scc.Add(w);
                } while (!w.Equals(v));

                results.Add(new ColoredGraph<V, E>.StronglyConnectedComponent(scc));
            }
        }
    }

    /// <summary>
    /// Implements the Tarjan algorithm for strongly connected components detection on directed non-colored graphs.
    /// </summary>
    public class TarjanAlgorithmProvider<V> : TarjanAlgorithmProvider<V, Byte>
        where V : IEquatable<V>
    {
        public TarjanAlgorithmProvider(Graph<V> graph) : base(graph) { }
    }

    /// <summary>
    /// Implements a depth-first search algorithm for connected components detection on a graph.
    /// </summary>
    public class DepthFirstCcdAlgorithmProvider
    {
        public static DepthFirstCcdAlgorithmProvider<V, E> Create<V, E>(ColoredGraph<V, E> graph)
            where V : IEquatable<V>
            where E : IEquatable<E>
        {
            return new DepthFirstCcdAlgorithmProvider<V, E>(graph);
        }

        public static DepthFirstCcdAlgorithmProvider<V> Create<V>(Graph<V> graph)
            where V : IEquatable<V>
        {
            return new DepthFirstCcdAlgorithmProvider<V>(graph);
        }
    }

    /// <summary>
    /// Implements a depth-first search algorithm for connected components detection on a graph.
    /// </summary>
    public class DepthFirstCcdAlgorithmProvider<V, E> : DepthFirstCcdAlgorithmProvider, IConnectedComponentsProvider<V, E>
        where V : IEquatable<V>
        where E : IEquatable<E>
    {
        private Dictionary<ColoredGraph<V, E>.Vertex, Int32> data;
        private Int32 index;
        protected ColoredGraph<V, E> graph;

        public DepthFirstCcdAlgorithmProvider(ColoredGraph<V, E> graph)
        {
            this.graph = graph;
        }

        public IEnumerable<ColoredGraph<V, E>.ConnectedComponent> Compute()
        {
            this.InitializeData();

            foreach(var v in graph.Vertices)
                if (data[v] < 0)
                {
                    this.Connect(v);
                    index++;
                }

            List<ColoredGraph<V, E>.Vertex>[] connectedComponents = new List<ColoredGraph<V,E>.Vertex>[index];
            for(Int32 i = 0; i < index; i++)
                connectedComponents[i] = new List<ColoredGraph<V,E>.Vertex>();

            foreach(var v in graph.Vertices)
                connectedComponents[data[v]].Add(v);

            this.DisposeData();

            return from rawComponent in connectedComponents
                   select new ColoredGraph<V, E>.ConnectedComponent(rawComponent);
        }

        private void InitializeData()
        {
            index = 0;
            data = new Dictionary<ColoredGraph<V, E>.Vertex, Int32>();
            foreach (var v in graph.Vertices)
                data.Add(v, -1);
        }

        private void DisposeData()
        {
            data.Clear();
        }

        private void Connect(ColoredGraph<V, E>.Vertex v)
        {
            if (data[v] >= 0)
                return;

            data[v] = index;
            foreach(var w in graph.GetNeighbourhoodSet(v))
                if (data[w] < 0)
                    this.Connect(w);
        }
    }

    /// <summary>
    /// Implements a depth-first search algorithm for connected components detection on a non-colored graph.
    /// </summary>
    public class DepthFirstCcdAlgorithmProvider<V> : DepthFirstCcdAlgorithmProvider<V, Byte>
        where V : IEquatable<V>
    {
        public DepthFirstCcdAlgorithmProvider(Graph<V> graph) : base(graph) { }
    }
}
