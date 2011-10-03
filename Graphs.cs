using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using Hamlet.Algorithms.Connectivity;
using Hamlet.Algorithms.ShortestPaths;

namespace Hamlet
{
    using Hamlet.Extensions;

    [Flags]
    public enum EdgeEnds
    {
        Start = 1,
        End = 2,
        Both = 3
    }

    public interface IVertex
    {
        IEnumerable<IVertex> Children { get; }
        IEnumerable<IVertex> Parents { get; }
    }

    public class GraphException : Exception
    {
        public GraphException(String message) : base(message) { }
    }

    /// <summary>
    /// Represents a generic colored graph.
    /// </summary>
    /// <typeparam name="V">This is the type of data contained in the graph's vertices. It must implement IEquatable(Of V).</typeparam>
    /// <typeparam name="E">This is the type of data associated to the graph's edges. It must implement IComparable(Of E).</typeparam>
    public class ColoredGraph<V, E>
        where V : IEquatable<V> 
        where E : IEquatable<E>
    {

        /// <summary>
        /// Represents a vertex.
        /// </summary>
        [DebuggerDisplay("{ToString()}")]
        public class Vertex : IEquatable<Vertex>, IVertex 
        {
            /// <summary>
            /// Gets the index of this vertex in the vertex collection.
            /// </summary>
            public Int32 Index { get; internal set; }

            /// <summary>
            /// Gets or sets the data contained in this vertex.
            /// </summary>
            public V Data { get; set; }

            internal List<Edge> OutgoingEdgesList;
            internal List<Edge> IncomingEdgesList;

            /// <summary>
            /// Gets a collection of all the edges departing from this vertex.
            /// </summary>
            public ReadOnlyCollection<Edge> OutgoingEdges { get; private set; }
            /// <summary>
            /// Gets a collection of all the edges ending on this vertex.
            /// </summary>
            public ReadOnlyCollection<Edge> IncomingEdges { get; private set; }
            /// <summary>
            /// Returns a set of this vertex's children, i.e. the vertices at the end of every outgoing edge.
            /// </summary>
            public IEnumerable<Vertex> Children
            {
                get
                {
                    foreach (Edge edge in this.OutgoingEdges)
                        yield return edge.End;
                }
            }
            /// <summary>
            /// Returns a set of this vertex's parents, i.e. the vertices at the start of every incoming edge.
            /// </summary>
            public IEnumerable<Vertex> Parents
            {
                get
                {
                    foreach (Edge edge in this.IncomingEdges)
                        yield return edge.Start;
                }
            }

            internal Vertex()
            {
                this.OutgoingEdgesList = new List<Edge>();
                this.IncomingEdgesList = new List<Edge>();
                this.OutgoingEdges = new ReadOnlyCollection<Edge>(this.OutgoingEdgesList);
                this.IncomingEdges = new ReadOnlyCollection<Edge>(this.IncomingEdgesList);
            }

            /// <summary>
            /// Checks whether this vertex is adjacent to a given vertex.
            /// </summary>
            public Boolean IsAdjacentTo(Vertex vertex)
            {
                return (this.OutgoingEdgesList.Where(edge => edge.End.Equals(vertex)).Count() > 0)
                    || (vertex.OutgoingEdgesList.Where(edge => edge.End.Equals(this)).Count() > 0);
            }

            public override string ToString()
            {
                return '{' + this.Data.ToString() + ", " + this.Index + '}';
            }

            public bool Equals(Vertex other)
            {
                if (other == null)
                    return false;
                return (this.Index == other.Index) && this.Data.Equals(other.Data);
            }

            IEnumerable<IVertex> IVertex.Children
            {
                get { return this.Children; }
            }

            IEnumerable<IVertex> IVertex.Parents
            {
                get { return this.Parents; }
            }
        }

        /// <summary>
        /// Represents an edge.
        /// </summary>
        [DebuggerDisplay("{ToString()}")]
        public class Edge : IEquatable<Edge>
        {
            /// <summary>
            /// Gets or sets the starting vertex of this edge.
            /// </summary>
            public Vertex Start { get; set; }
            /// <summary>
            /// Gets or sets the ending vertex of this edge.
            /// </summary>
            public Vertex End { get; set; }
            /// <summary>
            /// Gets or sets the data associated to this edge.
            /// </summary>
            public E Data { get; set; }

            internal Edge() { }

            /// <summary>
            /// Checks whether this edge is a loop.
            /// </summary>
            public Boolean IsLoop()
            {
                return this.Start.Equals(this.End);
            }

            /// <summary>
            /// Checks whether this edge is adjacent to a given edge.
            /// </summary>
            public Boolean IsAdjacentTo(Edge edge)
            {
                return (this.Start.Equals(edge.End)) || (this.End.Equals(edge.Start));
            }

            /// <summary>
            /// Inverts the versus of this edge, switching its ends.
            /// </summary>
            public void Reverse()
            {
                this.Start.OutgoingEdgesList.Remove(this);
                this.End.IncomingEdgesList.Remove(this);

                Vertex temp = this.Start;
                this.Start = this.End;
                this.End = temp;

                this.Start.OutgoingEdgesList.Add(this);
                this.End.IncomingEdgesList.Add(this);
            }

            public override string ToString()
            {
                return String.Format("{0} : {1} -> {2}", this.Data.ToString(), this.Start.ToString(), this.End.ToString());
            }

            public bool Equals(Edge other)
            {
                if (other == null)
                    return false;
                return (this.Start == other.Start) && (this.End == other.End) && this.Data.Equals(other.Data);
            }
        }

        public class StronglyConnectedComponent : ReadOnlyCollection<Vertex>
        {
            internal StronglyConnectedComponent(IList<Vertex> source) : base(source) { }
        }

        public class ConnectedComponent : ReadOnlyCollection<Vertex>
        {
            internal ConnectedComponent(IList<Vertex> source) : base(source) { }
        }

        protected List<Vertex> vertices;
        protected List<Edge> edges;
        protected IStronglyConnectedComponentsProvider<V, E> defaultSccProvider;
        protected IConnectedComponentsProvider<V, E> defaultCcProvider;

        protected IConnectedComponentsProvider<V, E> DefaultCCProvider
        {
            get
            {
                if (defaultCcProvider == null)
                    defaultCcProvider = DepthFirstCcdAlgorithmProvider.Create(this);
                return defaultCcProvider;
            }
        }

        protected IStronglyConnectedComponentsProvider<V, E> DefaultSCCProvider
        {
            get
            {
                if (defaultSccProvider == null)
                    defaultSccProvider = TarjanAlgorithmProvider.Create(this);
                return defaultSccProvider;
            }
        }

        /// <summary>
        /// Rerturns the collection of all data exposed by vertices, sorted in the same order as the vertices themselves.
        /// </summary>
        public IEnumerable<V> VerticesData
        {
            get
            {
                foreach (Vertex v in this.Vertices)
                    yield return v.Data;
            }
        }

        /// <summary>
        /// Returns the collection of all data exposed by edges.
        /// </summary>
        public IEnumerable<E> EdgesData
        {
            get
            {
                foreach (Edge e in this.Edges)
                    yield return e.Data;
            }
        }

        /// <summary>
        /// Gets the order of the graph, i.e. the number of its vertices.
        /// </summary>
        public Int32 Order
        {
            get { return this.vertices.Count; }
        }

        /// <summary>
        /// Gets the size of the graph, i.e. the numbers of its edges.
        /// </summary>
        public Int32 Size
        {
            get { return this.edges.Count; }
        }

        /// <summary>
        /// Determines if this graph is trivial, i.e. it has only 1 vertex.
        /// </summary>
        public Boolean IsTrivial
        {
            get { return this.vertices.Count == 1; }
        }
        
        /// <summary>
        /// Gets or sets a value indicating whether the graph is directed.
        /// </summary>
        public Boolean IsDirected { get; set; }

        /// <summary>
        /// Exposes the collection of all vertices.
        /// </summary>
        public ReadOnlyCollection<Vertex> Vertices { get; protected set; }

        /// <summary>
        /// Exposes the collection of all edges.
        /// </summary>
        public ReadOnlyCollection<Edge> Edges { get; protected set; }

        /// <summary>
        /// Initializes a new empty graph.
        /// </summary>
        public ColoredGraph()
        {
            vertices = new List<Vertex>();
            edges = new List<Edge>();
            this.Vertices = vertices.AsReadOnly();
            this.Edges = edges.AsReadOnly();
        }

        protected void SetVertices(List<Vertex> vertices)
        {
            this.vertices = vertices;
            this.Vertices = vertices.AsReadOnly();
        }

        protected void SetEdges(List<Edge> edges)
        {
            this.edges = edges;
            this.Edges = edges.AsReadOnly();
        }

        /// <summary>
        /// Adds a new vertex to the graph
        /// </summary>
        /// <param name="data">The data contained the vertex.</param>
        public virtual Vertex AddVertex(V data)
        {
            Vertex vertex = new Vertex() { Data = data, Index = vertices.Count };
            vertices.Add(vertex);
            return vertex;
        }

        /// <summary>
        /// Adds new vertices to the graph.
        /// </summary>
        /// <param name="data">A collection of values to assign to each vertex added.</param>
        public virtual void AddVertices(params V[] data)
        {
            foreach (V vertexData in data)
                this.AddVertex(vertexData);
        }

        /// <summary>
        /// Removes a vertex from the graph, provided its index.
        /// </summary>
        /// <param name="index">Indicates the index of the vertex being removed.</param>
        public virtual void RemoveVertex(Int32 index)
        {
            if (index < 0 || index >= vertices.Count)
                throw new IndexOutOfRangeException();
            this.RemoveVertices(new Vertex[] { vertices[index] });
        }

        /// <summary>
        /// Removes a vertex from the graph.
        /// </summary>
        /// <param name="vertex">The vertex being removed.</param>
        public virtual void RemoveVertex(Vertex vertex)
        {
            this.RemoveVertices(new Vertex[] { vertex });
        }

        /// <summary>
        /// Removes all the vertices that match a given content from the graph.
        /// </summary>
        /// <param name="data">Indicates the data a vertex must posses to be deleted from the graph.</param>
        public virtual void RemoveVertices(V data)
        {
            this.RemoveVertices((from v in vertices where v.Data.Equals(data) select v).ToArray());
        }

        /// <summary>
        /// Removes a range of vertices.
        /// </summary>
        /// <param name="startIndex">The vertex index at which to start removing vertices.</param>
        /// <param name="count">The number of vertices being removed.</param>
        public virtual void RemoveVertices(Int32 startIndex, Int32 count)
        {
            this.RemoveVertices((from v in vertices where (v.Index >= startIndex && v.Index < startIndex + count) select v).ToArray());
        }

        /// <summary>
        /// Deletes all the vertices and all the edges from the graph.
        /// </summary>
        public virtual void Clear()
        {
            vertices.Clear();
            edges.Clear();
        }

        /// <summary>
        /// Creates a new edge linking two existing vertices.
        /// </summary>
        /// <param name="vertexIndex1">Starting vertex.</param>
        /// <param name="vertexIndex2">Ending vertex.</param>
        /// <param name="data">Data associated to the edge.</param>
        public virtual Edge AddEdge(Vertex vertex1, Vertex vertex2, E data)
        {
            Edge edge = new Edge() { Start = vertex1, End = vertex2, Data = data };
            edges.Add(edge);
            vertex1.OutgoingEdgesList.Add(edge);
            vertex2.IncomingEdgesList.Add(edge);
            return edge;
        }

        /// <summary>
        /// Creates a new edge linking two existing vertices.
        /// </summary>
        /// <param name="vertexIndex1">Index of the starting vertex.</param>
        /// <param name="vertexIndex2">Index of the ending vertex.</param>
        /// <param name="data">Data associated to the edge.</param>
        public Edge AddEdge(Int32 vertexIndex1, Int32 vertexIndex2, E data)
        {
            this.CheckVertexIndices(vertexIndex1, vertexIndex2);
            return this.AddEdge(vertices[vertexIndex1], vertices[vertexIndex2], data);
        }

        /// <summary>
        /// Removes an existing edge between two vertices. If there are more than one edge in between the vertices,
        /// the method will remove all of them. If the graph is directed, only edges going straight from the first
        /// to the second will be considered.
        /// </summary>
        /// <param name="vertexIndex1">Starting vertex.</param>
        /// <param name="vertexIndex2">Ending vertex.</param>
        public virtual void RemoveEdge(Vertex vertex1, Vertex vertex2)
        {
            var matches = vertex1.OutgoingEdgesList.Intersect(vertex2.IncomingEdgesList).ToArray();
            foreach (var edge in matches)
            {
                vertex1.OutgoingEdgesList.Remove(edge);
                vertex2.IncomingEdgesList.Remove(edge);
                edges.Remove(edge);
            }

            if (!this.IsDirected)
                this.RemoveEdge(vertex2, vertex1);
        }

        /// <summary>
        /// Removes an existing edge between two vertices. If there are more than one edge in between the vertices,
        /// the method will remove all of them. If the graph is directed, only edges going straight from the first
        /// to the second will be considered.
        /// </summary>
        /// <param name="vertexIndex1">Index of the starting vertex.</param>
        /// <param name="vertexIndex2">Index of the ending vertex.</param>
        public void RemoveEdge(Int32 vertexIndex1, Int32 vertexIndex2)
        {
            this.CheckVertexIndices(vertexIndex1, vertexIndex2);
            this.RemoveEdge(vertices[vertexIndex1], vertices[vertexIndex2]);
        }

        /// <summary>
        /// Finds all the edges starting or ending with the specified vertex.
        /// </summary>
        /// <param name="ends">Flags indicating whether to considering incoming or outcoming edges, or both.</param>
        public IEnumerable<Edge> FindEdges(Vertex vertex, EdgeEnds ends)
        {
            if (ends.HasFlag(EdgeEnds.Start))
                foreach (Edge e in vertex.OutgoingEdgesList)
                    yield return e;

            if (ends.HasFlag(EdgeEnds.End))
                foreach (Edge e in vertex.IncomingEdgesList)
                    yield return e;
        }

        /// <summary>
        /// Finds all the edges starting at vertex1 and ending at vertex2.
        /// </summary>
        public IEnumerable<Edge> FindEdges(Vertex vertex1, Vertex vertex2, Boolean ignoreGraphDirection = false)
        {
            var result = from edge in vertex1.OutgoingEdgesList 
                         where edge.End.Equals(vertex2) 
                         select edge;

            if (ignoreGraphDirection || !this.IsDirected)
                result = result.Concat(
                    from edge in vertex2.OutgoingEdgesList
                    where edge.End.Equals(vertex1)
                    select edge);

            return result;
        }

        /// <summary>
        /// Finds all the edges starting at the vertex whose index is vertexIndex1 and ending at the vertex whose index is vertexIndex2.
        /// </summary>
        public IEnumerable<Edge> FindEdges(Int32 vertexIndex1, Int32 vertexIndex2, Boolean ignoreGraphDirection = false)
        {
            this.CheckVertexIndices(vertexIndex1, vertexIndex2);
            return this.FindEdges(vertices[vertexIndex1], vertices[vertexIndex2], ignoreGraphDirection);
        }

        protected void RemoveVertices(IEnumerable<Vertex> matches)
        {
            foreach (var vertex in matches)
                this.RemoveVertexAndEdges(vertex);
            this.RefreshVertexIndices();
        }

        protected virtual void RemoveVertexAndEdges(Vertex vertex)
        {
            vertices.Remove(vertex);

            foreach (Edge edge in vertex.IncomingEdgesList)
            {
                edge.Start.OutgoingEdgesList.Remove(edge);
                edges.Remove(edge);
            }
            vertex.IncomingEdgesList.Clear();

            foreach (Edge edge in vertex.OutgoingEdgesList)
            {
                edge.End.IncomingEdgesList.Remove(edge);
                edges.Remove(edge);
            }
            vertex.OutgoingEdgesList.Clear();
        }

        private void RefreshVertexIndices()
        {
            for (Int32 i = 0; i < vertices.Count; i++)
                vertices[i].Index = i;
        }

        private void CheckVertexIndices(params Int32[] indices)
        {
            foreach (Int32 index in indices)
                if (index < 0 || index >= vertices.Count)
                    throw new IndexOutOfRangeException();
        }

        /// <summary>
        /// Checks whether this graph is a multigraph.
        /// </summary>
        public Boolean IsMultigraph()
        {
            for (Int32 i = 0; i < edges.Count; i++)
            {
                if (edges[i].IsLoop())
                    return true;
                for (Int32 j = i + 1; j < edges.Count; j++)
                    if (edges[i].End.Equals(edges[j].End))
                        return true;
            }
            return false;
        }

        /// <summary>
        /// Calculates and returns the adjacency matrix for this graph.
        /// </summary>
        public Byte[,] GetAdjacencyMatrix()
        {
            Byte[,] matrix = new Byte[this.Order, this.Order];

            foreach (Edge e in edges)
            {
                matrix[e.Start.Index, e.End.Index] = 1;
                if (this.IsDirected)
                    matrix[e.End.Index, e.Start.Index] = 1;
            }

            return matrix;
        }

        /// <summary>
        /// Retrieves the neighbourhood set of a given vertex.
        /// </summary>
        public IEnumerable<Vertex> GetNeighbourhoodSet(Vertex vertex)
        {
            if (this.IsDirected)
                return this.FindEdges(vertex, EdgeEnds.Start)
                           .Select(e => e.End)
                           .Distinct()
                           .Where(v => !v.Equals(vertex));

            return this.FindEdges(vertex, EdgeEnds.Both)
                       .SelectMany(e => new Vertex[] { e.Start, e.End })
                       .Distinct()
                       .Where(v => !v.Equals(vertex));
        }

        /// <summary>
        /// Gets the degree (i.e. the cardinality of neighbourhood set) of a given vertex.
        /// </summary>
        public Int32 GetDegree(Vertex vertex)
        {
            return this.GetNeighbourhoodSet(vertex).Count();
        }

        /// <summary>
        /// Checks whether this graph is subgraph of another graph.
        /// </summary>
        public Boolean IsSubgraphOf(ColoredGraph<V, E> superGraph)
        {
            return this.vertices.All(v => superGraph.vertices.Contains(v)) &&
                   this.edges.All(e => superGraph.edges.Contains(e));
        }

        /// <summary>
        /// Check whether this graph spans another graph.
        /// </summary>
        public Boolean Spans(ColoredGraph<V, E> otherGraph)
        {
            return this.vertices.SequenceEqual(otherGraph.vertices);
        }

        /// <summary>
        /// Checks whether this graph is complete, i.e. all vertices are adjacent.
        /// </summary>
        public Boolean IsComplete()
        {
            for (Int32 i = 0; i < vertices.Count; i++)
                for (Int32 j = i + 1; j < vertices.Count; j++)
                    if (!vertices[i].IsAdjacentTo(vertices[j]))
                        return false;
            return true;
        }

        /// <summary>
        /// Checks whether this graph is regular, i.e. the degree of all vertices is the same.
        /// </summary>
        public Boolean IsRegular()
        {
            if (vertices.Count == 0)
                return true;

            Int32 degree = this.GetDegree(vertices[0]);
            return vertices.All(v => this.GetDegree(v) == degree);
        }

        /// <summary>
        /// Checks whether this graph is bipartite, given a function to classify its vertices in two sets (X and Y).
        /// </summary>
        /// <param name="selector">Selector function that returns true if the data of a given vertex belongs to set X, or false otherwise.</param>
        public Boolean IsBipartite(Func<V, Boolean> selector)
        {
            foreach (Edge e in edges)
                if ((selector(e.Start.Data) && selector(e.End.Data)) || (!selector(e.End.Data) && !selector(e.Start.Data)))
                    return false;
            return true;
        }

        /// <summary>
        /// Checks whether this graph is acyclic, i.e. there are no strongly connected components but its
        /// simple vertices. To detected strongly connected components, a default implementation of the
        /// Tarjan algorithm is used.
        /// </summary>
        public Boolean IsAcyclic()
        {
            return this.IsAcyclic(this.DefaultSCCProvider);
        }

        /// <summary>
        /// Checks whether this graph is acyclic, i.e. there are no strongly connected components but its
        /// simple vertices. To detected strongly connected components, the method uses the given algorithm.
        /// </summary>
        /// <param name="sccProvider">The algorithm provider to use when calculating the strongly connected components.</param>
        public Boolean IsAcyclic(IStronglyConnectedComponentsProvider<V, E> sccProvider)
        {
            var scComponents = sccProvider.Compute();
            return scComponents.All(scc => scc.Count == 1);
        }

        /// <summary>
        /// Checks whether an edge is a bridge. This method counts the number of connected components
        /// with and without that edge and returns true if those number are different (the latter
        /// being greater than the first). For the computation, a default Depth-First search is used.
        /// </summary>
        public Boolean IsBridge(Edge edge)
        {
            return this.IsBridge(edge, this.DefaultCCProvider);
        }

        /// <summary>
        /// Checks whether an edge is a bridge. This method counts the number of connected components
        /// with and without that edge and returns true if those number are different (the latter
        /// being greater than the first). For the computation, the given algorithm is used.
        /// </summary>
        /// <param name="ccProvider">The algorithm provider to use when calculating the connected components.</param>
        public Boolean IsBridge(Edge edge, IConnectedComponentsProvider<V, E> ccProvider)
        {
            Int32 ccNumber = ccProvider.Compute().Count();
            this.RemoveEdge(edge.Start, edge.End);
            Int32 ccNumberWithoutEdge = ccProvider.Compute().Count();
            this.AddEdge(edge.Start, edge.End, edge.Data);
            return (ccNumberWithoutEdge > ccNumber);
        }

        /// <summary>
        /// Checks whether the graph is connected. In order to obtain the result, all the connected components of the
        /// graph are computed using a simple depth-first visiting algorithm.
        /// </summary>
        public Boolean IsConnected()
        {
            return this.IsConnected(this.DefaultCCProvider);
        }

        /// <summary>
        /// Checks whether the graph is connected. In order to obtain the result, all the connected components of the
        /// graph are computed using the specified algorithm.
        /// </summary>
        /// <param name="allPairsProvider">The algorithm provider to use when calculating all the shortest paths pairs.</param>
        public Boolean IsConnected(IConnectedComponentsProvider<V, E> connectedComponentsProvider)
        {
            var connectedComponents = connectedComponentsProvider.Compute();
            return connectedComponents.Count() == 1;
        }

        /// <summary>
        /// Checks whether the graph is weakly connected, i.e. if its undirected version is connected. In order to find
        /// the connected components a default depth-first algorithm is used.
        /// </summary>
        public Boolean IsWeaklyConnected()
        {
            return this.IsWeaklyConnected(this.DefaultCCProvider);
        }

        /// <summary>
        /// Checks whether the graph is weakly connected, i.e. if its undirected version is connected. In order to find
        /// the connected components the given algorithm is used.
        /// </summary>
        /// <param name="connectedComponentsProvider">The algorithm provider to use to find connected components</param>
        public Boolean IsWeaklyConnected(IConnectedComponentsProvider<V, E> connectedComponentsProvider)
        {
            Boolean prevValue = this.IsDirected;
            Boolean result;
            this.IsDirected = false;
            result = connectedComponentsProvider.Compute().Count() == 1;
            this.IsDirected = prevValue;
            return result;
        }

        /// <summary>
        /// Checks whether the graph is a tree, i.e. a connected acyclic graph.
        /// N.B.: it is not required that the graph is undirected since the default algorithm for cycles detection (Tarjan's algorithm)
        /// requires a directed graph. Nevertheless, a tree is always considered as undirected graph, even if its base is directed.
        /// </summary>
        public Boolean IsTree()
        {
            if (!this.IsDirected)
                throw new InvalidOperationException("The default algorithm for cycles detection requires a directed graph to operate on.");
            return this.IsAcyclic() && this.IsConnected();
        }

        /// <summary>
        /// Checks whether this graph is a tree, i.e. a connected acyclic graph. To test the existance of cycles and the connectivity
        /// properties the given algorithm providers are used.
        /// </summary>
        /// <param name="sccProvider">The algorithm provider to use when calculating the set of strongly connected components of the graph.</param>
        /// <param name="allPairsProvider">The algorithm provider to use when calculating all the shortest paths pairs.</param>
        /// <returns></returns>
        public Boolean IsTree(IStronglyConnectedComponentsProvider<V, E> sccProvider, IConnectedComponentsProvider<V, E> ccProvider)
        {
            return this.IsAcyclic(sccProvider) && this.IsConnected(ccProvider);
        }

        /// <summary>
        /// Converts this graph to a weighted graph where all weights are calculated with the given function. The result object represents an alternative way
        /// to access the same data and therefore all vertices and edges will be the same.
        /// </summary>
        public WeightedGraph<V, E> AsWeightedGraph(Func<E, Double> weightFunction)
        {
            WeightedGraph<V, E> result = new WeightedGraph<V, E>(weightFunction);
            result.SetVertices(this.vertices);
            result.SetEdges(this.edges);
            result.IsDirected = this.IsDirected;
            return result;
        }

        /// <summary>
        /// Converts this graph to a tree object (iff this graph has been verified to be a tree). The result object represents an alternative way
        /// to access the same data and therefore all vertices and edges will be the same.
        /// </summary>
        public ColoredTree<V, E> AsTree()
        {
            if (!this.IsTree())
                throw new InvalidCastException("This graph is not a tree.");
            ColoredTree<V, E> result = new ColoredTree<V, E>();
            result.SetVertices(this.vertices);
            result.SetEdges(this.edges);
            result.IsDirected = this.IsDirected;
            return result;
        }

        internal ColoredTree<V, E> ForceTree()
        {
            ColoredTree<V, E> result = new ColoredTree<V, E>();
            result.SetVertices(this.vertices);
            result.SetEdges(this.edges);
            result.IsDirected = this.IsDirected;

            foreach (var vertex in result.vertices)
                if (vertex.IncomingEdges.Count == 0)
                {
                    result.Root = vertex;
                    break;
                }

            return result;
        }

        /// <summary>
        /// Saves this graph to a text file using the given functions for trasforming vertices' and edges' data into strings.
        /// </summary>
        /// <param name="filename">Path of the file being written.</param>
        /// <param name="vertexToString">A conversion function that takes a V object and converts it to a string.</param>
        /// <param name="edgeToString">A conversion function that taks an E object and converts it to a string.</param>
        public void SaveToFile(String filename, Func<V, String> vertexToString, Func<E, String> edgeToString)
        {
            StreamWriter writer = new StreamWriter(filename);

            writer.WriteLine(vertices.Count);
            foreach (var vertex in vertices)
                writer.WriteLine(vertexToString(vertex.Data));

            writer.WriteLine(edges.Count);
            foreach (var edge in edges)
            {
                writer.Write(edge.Start.Index);
                writer.Write(' ');
                writer.Write(edge.End.Index);
                writer.Write(' ');
                writer.WriteLine(edgeToString(edge.Data));
            }

            writer.WriteLine(this.IsDirected ? 1 : 0);
            writer.Close();
        }

        /// <summary>
        /// Saves this graph to a text file using the given function for trasforming vertices' data into strings. Edges' data are converted with the default .ToString() method.
        /// </summary>
        /// <param name="filename">Path of the file being written.</param>
        /// <param name="vertexToString">A conversion function that takes a V object and converts it to a string.</param>
        public void SaveToFile(String filename, Func<V, String> vertexToString)
        {
            this.SaveToFile(filename, vertexToString, e => e.ToString());
        }

        /// <summary>
        /// Saves this graph to a text file using the given function for trasforming edges' data into strings. Vertices' data are converted with the default .ToString() method.
        /// </summary>
        /// <param name="filename">Path of the file being written.</param>
        /// <param name="edgeToString">A conversion function that taks an E object and converts it to a string.</param>
        public void SaveToFile(String filename, Func<E, String> edgeToString)
        {
            this.SaveToFile(filename, v => v.ToString(), edgeToString);
        }

        /// <summary>
        /// Saves this graph to a text file. Vertices' and edges' data are converted into strings with the default .ToString() method.
        /// </summary>
        /// <param name="filename">Path of the file being written.</param>
        public void SaveToFile(String filename)
        {
            this.SaveToFile(filename, v => v.ToString(), e => e.ToString());
        }

        /// <summary>
        /// Loads a graph from a text file using the given functions to parse vertices' and edges' data.
        /// </summary>
        /// <param name="filename">Path of the file being read.</param>
        /// <param name="stringToVertex">A conversion function that takes a string and outputs an object of type V.</param>
        /// <param name="stringToEdge">A conversion function that takes a string and outputs an object of type E.</param>
        public static ColoredGraph<V, E> LoadFromFile(String filename, Func<String, V> stringToVertex, Func<String, E> stringToEdge)
        {
            StreamReader reader = new StreamReader(filename);
            Int32 verticesCount, edgesCount;
            ColoredGraph<V, E> graph = new ColoredGraph<V, E>();

            try
            {
                verticesCount = Convert.ToInt32(reader.ReadLine());
                for (Int32 i = 0; i < verticesCount; i++)
                {
                    V data = stringToVertex(reader.ReadLine());
                    graph.AddVertex(data);
                }

                edgesCount = Int32.Parse(reader.ReadLine());
                for (Int32 i = 0; i < edgesCount; i++)
                {
                    String[] parts = reader.ReadLine().Split(' ');
                    Int32 vertexIndex1 = Convert.ToInt32(parts[0]);
                    Int32 vertexIndex2 = Convert.ToInt32(parts[1]);
                    E data = stringToEdge(parts[2]);
                    graph.AddEdge(graph.vertices[vertexIndex1], graph.vertices[vertexIndex2], data);
                }

                Int32 directed = Convert.ToInt32(reader.ReadLine());
                graph.IsDirected = (directed == 1);
            }
            catch
            {
                reader.Close();
                throw new ArgumentException("Some of the data in the graph are invalid or corrupted.");
            }
            finally
            {
                reader.Close();
            }

            return graph;
        }

        /// <summary>
        /// Loads a graph from a text file using the given function to parse vertices' data. Edges' data is parsed using the Convert.ChangeType method.
        /// </summary>
        /// <param name="filename">Path of the file being read.</param>
        /// <param name="stringToVertex">A conversion function that takes a string and outputs an object of type V.</param>
        public static ColoredGraph<V, E> LoadFromFile(String filename, Func<String, V> stringToVertex)
        {
            return LoadFromFile(filename, stringToVertex, str => (E)Convert.ChangeType(str, typeof(E)));
        }

        /// <summary>
        /// Loads a graph from a text file using the given function to parse edges' data. Vertices' data is parsed using the Convert.ChangeType method.
        /// </summary>
        /// <param name="filename">Path of the file being read.</param>
        /// <param name="stringToVertex">A conversion function that takes a string and outputs an object of type V.</param>
        /// <param name="stringToEdge">A conversion function that takes a string and outputs an object of type E.</param>
        public static ColoredGraph<V, E> LoadFromFile(String filename, Func<String, E> stringToEdge)
        {
            return LoadFromFile(filename, str => (V)Convert.ChangeType(str, typeof(V)), stringToEdge);
        }

        /// <summary>
        /// Loads a graph from a text file, trying to parse vertices' and edges' data with the default Convert.ChangeType method.
        /// </summary>
        /// <param name="filename">Path of the file being read.</param>
        public static ColoredGraph<V, E> LoadFromFile(String filename)
        {
            return LoadFromFile(filename, str => (V)Convert.ChangeType(str, typeof(V)), str => (E)Convert.ChangeType(str, typeof(E)));
        }
    }

    /// <summary>
    /// Represents a graph without colored edges (edges do not expose significant informations).
    /// </summary>
    /// <typeparam name="V">This is the type of data contained in the graph's vertices. It must implement IEquatable(Of V).</typeparam>
    public class Graph<V> : ColoredGraph<V, Byte>
        where V : IEquatable<V>
    {
        /// <summary>
        /// Creates a new edge linking two existing vertices.
        /// </summary>
        /// <param name="vertexIndex1">Starting vertex.</param>
        /// <param name="vertexIndex2">Ending vertex.</param>
        public Edge AddEdge(Vertex vertex1, Vertex vertex2)
        {
            return base.AddEdge(vertex1, vertex2, 0);
        }

        public new Tree<V> AsTree()
        {
            return base.AsTree() as Tree<V>;
        }

        public static Graph<V> InferFrom(ColoredGraph<V, Byte> graph)
        {
            Graph<V> result = new Graph<V>();
            result.SetVertices(graph.Vertices.ToList());
            result.SetEdges(graph.Edges.ToList());
            result.IsDirected = graph.IsDirected;
            return result;
        }
    }

    /// <summary>
    /// Represents a weighted graph, i.e. a colored graph where the color function for the edges returns
    /// a numeric value.
    /// </summary>
    /// <typeparam name="V">This is the type of data contained in the graph's vertices. It must implement IEquatable(Of V).</typeparam>
    /// <typeparam name="E">This is the type of data associated to the graph's edges. It must implement IComparable(Of E).</typeparam>
    public class WeightedGraph<V, E> : ColoredGraph<V, E>
        where V : IEquatable<V>
        where E : IEquatable<E>
    {
        /// <summary>
        /// Represents a walk on a graph.
        /// </summary>
        public class Walk : ReadOnlyCollection<Edge>, IEquatable<Walk>
        {
            private static readonly CollectionComparer<Edge> defaultComparer = new CollectionComparer<Edge>();

            /// <summary>
            /// Indicates if the walk is closed, i.e. the first and last vertices are equals.
            /// </summary>
            public Boolean IsClosed { get; private set; }
            /// <summary>
            /// Indicates if the walk is a cycle.
            /// </summary>
            public Boolean IsCycle { get; private set; }
            /// <summary>
            /// Indicates if the walk is a path.
            /// </summary>
            public Boolean IsPath { get; private set; }
            /// <summary>
            /// Indicates if a path is trivial.
            /// </summary>
            public Boolean IsTrivialPath { get; private set; }

            /// <summary>
            /// Retrieves a collection of all vertices in the walk.
            /// </summary>
            public IEnumerable<Vertex> Vertices
            {
                get
                {
                    foreach (Edge e in this)
                        yield return e.Start;
                    if (this.Count > 0)
                        yield return this.Last().End;
                }
            }

            /// <summary>
            /// Creates a walk from a succession of adjacent edges.
            /// </summary>
            public Walk(IList<Edge> ed) : base(ed)
            {
                if (ed.Count > 0)
                {
                    this.IsClosed = ed.First().Equals(ed.Last());

                    var vertices = ed.Select(e => e.Start).ToList();
                    vertices.Add(ed.Last().End);

                    this.IsPath = true;
                    this.IsCycle = true;
                    for (Int32 i = 0; i < vertices.Count && (this.IsPath || this.IsCycle); i++)
                        for (Int32 j = i + 1; j < vertices.Count; j++)
                            if ((i != j) && (vertices[i].Equals(vertices[j])))
                            {
                                this.IsPath = false;
                                if (!((i == 0 && j == vertices.Count - 1) || (j == 0 && i == vertices.Count - 1)))
                                    this.IsCycle = false;

                                if (!this.IsPath && !this.IsCycle)
                                    break;
                            }
                    this.IsCycle &= this.IsClosed;
                    this.IsTrivialPath = false;
                }
                else
                {
                    this.IsClosed = this.IsPath = this.IsCycle = this.IsTrivialPath = true;
                }
            }

            /// <summary>
            /// Check whether two walks intersect.
            /// </summary>
            public Boolean Intersects(Walk otherWalk)
            {
                return this.Vertices.Intersect(otherWalk.Vertices).Count() > 0;
            }

            /// <summary>
            /// Check whether two walks are independent, i.e. if they have no vertices in common except, at most, the end.
            /// </summary>
            public Boolean IsIndependentOf(Walk otherWalk)
            {
                var commonVertices = this.Vertices.Intersect(otherWalk.Vertices);
                Int32 count = commonVertices.Count();

                return (count == 0) || (count == 1 && commonVertices.Contains(this.Last().End));
            }

            public bool Equals(Walk other)
            {
                return defaultComparer.Equals(this, other);
            }
        }

        /// <summary>
        /// Contains an anonymous function that computes the weight of an edge based on its data.
        /// </summary>
        public Func<Edge, Double> EdgeWeight { get; protected set; }

        /// <summary>
        /// Contains an anonymous function that computs the weight of an object of type E.
        /// </summary>
        public Func<E, Double> Weight { get; protected set; }

        /// <summary>
        /// Initializes a new empty weighted graph specifying a new weight function.
        /// </summary>
        /// <param name="weightFunction">A weight function that, given the edge data, calculates its weight as a floating point value.</param>
        public WeightedGraph(Func<E, Double> weightFunction)
        {
            this.Weight = weightFunction;
            this.EdgeWeight = edge => weightFunction(edge.Data);
        }
    }
}
