using System;
using System.Collections.Generic;
using System.Linq;

namespace Hamlet
{
    using Hamlet.Extensions;

    /// <summary>
    /// Represents a colored tree, i.e. an undirected acyclic connected edge-colored graph.
    /// </summary>
    public class ColoredTree<V, E> : ColoredGraph<V, E>
        where V : IEquatable<V>
        where E : IEquatable<E>
    {
        private Vertex root;
        private Dictionary<Vertex, Boolean> reshapeFlag;

        /// <summary>
        /// Gets or sets the root vertex.
        /// </summary>
        public Vertex Root
        {
            get { return root; }
            set
            {
                if (value.IncomingEdges.Count > 0)
                    throw new ArgumentException("The root vertex should not have any incoming edges.");
                root = value;
                this.ReshapeBranchDirections();
            }
        }

        // When the root changes, the directions of edges changes as well
        private void ReshapeBranchDirections()
        {
            reshapeFlag = vertices.ToDictionary(vertex => vertex, vertex => false);
            if (root.IncomingEdges.Count > 0)
                root.IncomingEdges
                    .ToArray()
                    .Do(edge => { edge.Reverse(); });
            this.ReshapeBranchDirections(root);
            reshapeFlag.Clear();
            reshapeFlag = null;
        }

        private void ReshapeBranchDirections(Vertex root)
        {
            if (reshapeFlag[root])
                return;

            reshapeFlag[root] = true;

            root.Children
                .Where(child => child.IncomingEdges.Count > 1)
                .SelectMany(child => child.IncomingEdges)
                .Where(childIncomingEdge => !root.OutgoingEdges.Contains(childIncomingEdge))
                .ToArray()
                .Do(wrongEdge => { wrongEdge.Reverse(); });

            root.Children
                .ToArray()
                .Do(child => { ReshapeBranchDirections(child); });
        }

        /// <summary>
        /// Checks whether the tree is rooted (a root exists).
        /// </summary>
        public Boolean IsRooted
        {
            get { return this.Root != null; }
        }

        /// <summary>
        /// Initializes a new empty tree.
        /// </summary>
        public ColoredTree() : base()
        {
            this.IsDirected = true;
        }

        /// <summary>
        /// Adds a new leaf to the tree.
        /// </summary>
        /// <param name="parent">Parent vertex of the new leaf.</param>
        /// <param name="vertexData">Data contained in the new leaf.</param>
        /// <param name="edgeData">Data contained in the edge linking the leaf to its parent.</param>
        public virtual Vertex AddLeaf(Vertex parent, V vertexData, E edgeData)
        {
            var addedVertex = base.AddVertex(vertexData);
            base.AddEdge(parent, addedVertex, edgeData);
            return addedVertex;
        }

        /// <summary>
        /// Creates a new root for this tree.
        /// </summary>
        /// <param name="data">Data to be attached to the root vertex being created.</param>
        public void CreateRoot(V data)
        {
            if (this.IsRooted)
                throw new InvalidOperationException("Cannot add more than one root to a tree.");
            root = new Vertex();
            root.Index = 0;
            root.Data = data;
            vertices.Add(root);
        }

        protected override void RemoveVertexAndEdges(Vertex vertex)
        {
            foreach (var edge in vertex.OutgoingEdges)
                base.RemoveVertexAndEdges(edge.End);
            base.RemoveVertexAndEdges(vertex);
        }

        /// <summary>
        /// Forbidden method in a Tree.
        /// </summary>
        public override Vertex AddVertex(V data)
        {
            throw new InvalidOperationException("Cannot add unconnected vertices in a tree. Use AddLeaf instead.");
        }

        /// <summary>
        /// Forbidden method in a Tree.
        /// </summary>
        public override void AddVertices(params V[] data)
        {
            throw new InvalidOperationException("Cannot add unconnected vertices in a tree. Use AddLeaf instead.");
        }

        /// <summary>
        /// Forbidden method in a Tree.
        /// </summary>
        public override Edge AddEdge(ColoredGraph<V, E>.Vertex vertex1, ColoredGraph<V, E>.Vertex vertex2, E data)
        {
            throw new InvalidOperationException("Cannot add arbitrary edges in a tree. Use AddLeaf to connected an existing vertex to a new leaf.");
        }

        /// <summary>
        /// Gets the height of the tree.
        /// </summary>
        public Int32 GetHeight()
        {
            this.CheckRoot(true);
            return this.GetHeight(this.Root);
        }

        private Int32 GetHeight(Vertex vertex)
        {
            if (vertex.OutgoingEdges.Count == 0)
                return 0;
            return 1 + vertex.OutgoingEdges.Select(edge => this.GetHeight(edge.End)).Max();
        }

        private class TreeVertexData : IComparable<TreeVertexData>
        {
            public Int32 Label { get; set; }
            public List<Int32> OrderedLabels { get; set; }
            public List<IVertex> OrderedChildren { get; set; }

            public TreeVertexData()
            {
                this.Label = 0;
                this.OrderedLabels = new List<int>();
                this.OrderedChildren = new List<IVertex>();
            }

            public override String ToString()
            {
                return String.Format("Label: {0}, OrderedLabels = [{1}], OrderedChildren = [{2}]",
                    this.Label,
                    this.OrderedLabels.Aggregate(String.Empty, (str, num) => str + ", " + num).Trim(',', ' '),
                    this.OrderedChildren.Aggregate(String.Empty, (str, v) => str + ", " + v).Trim(',', ' '));
            }

            public Int32 CompareTo(TreeVertexData other)
            {
                if ((this.OrderedLabels.Count == 0) || (other.OrderedLabels.Count == 0))
                    return this.OrderedLabels.Count.CompareTo(other.OrderedLabels.Count);

                Int32 minLength = Math.Min(this.OrderedLabels.Count, other.OrderedLabels.Count);

                for (Int32 k = 0; k < minLength; k++)
                {
                    if (this.OrderedLabels[k] > other.OrderedLabels[k])
                        return 1;
                    if (this.OrderedLabels[k] < other.OrderedLabels[k])
                        return -1;
                }

                return 0;
            }
        }

        /// <summary>
        /// Checks whether two (rooted) trees are isomorphic.
        /// </summary>
        public Boolean IsIsomorphicTo<H, K>(ColoredTree<H, K> tree)
            where H : IEquatable<H>
            where K : IEquatable<K>
        {
            Int32 myHeight = this.GetHeight();
            Int32 treeHeight = tree.GetHeight();

            if (myHeight != treeHeight)
                return false;

            List<IVertex>[] levels = ArrayCreator<List<IVertex>>.Create(myHeight + 1);

            levels[0].Add(this.Root);
            levels[0].Add(tree.Root);
            for (Int32 i = 1; i < levels.Length; i++)
                levels[i].AddRange(levels[i - 1].SelectMany(vertex => vertex.Children));

            Dictionary<IVertex, TreeVertexData> data = new Dictionary<IVertex, TreeVertexData>();
            foreach (var v in this.Vertices)
                data[v] = new TreeVertexData();
            foreach (var v in tree.Vertices)
                data[v] = new TreeVertexData();

            CollectionComparer<Int32> intCollectionComparer = new CollectionComparer<Int32>();
            for (Int32 i = myHeight - 1; i >= 0; i--)
            {
                for (Int32 j = 0; j < levels[i + 1].Count; j++)
                {
                    var v = levels[i + 1][j];
                    var parent = v.Parents.First();
                    data[parent].OrderedLabels.Add(data[v].Label);
                    data[parent].OrderedChildren.Add(v);
                }

                levels[i].Sort((x, y) => data[x].CompareTo(data[y]));

                foreach (var v in levels[i])
                {
                    data[v].Label =
                        levels[i]
                        .Select(vertex => data[vertex].OrderedLabels)
                        .Distinct(intCollectionComparer)
                        .IndexOf(data[v].OrderedLabels, intCollectionComparer);
                }
            }

            return data[this.Root].Label == data[tree.Root].Label;
        }

        private void CheckRoot(Boolean launchExcpetion = false)
        {
            if (this.IsRooted)
                return;

            foreach (Vertex vertex in this.Vertices)
                if (vertex.IncomingEdges.Count == 0)
                {
                    root = vertex;
                    return;
                }

            if (launchExcpetion)
                throw new InvalidOperationException("A rooted tree is required for this operation.");
        }
    }

    /// <summary>
    /// Represents a tree, i.e. an undirected acyclic connected non-edge-colored graph.
    /// </summary>
    public class Tree<V> : ColoredTree<V, Byte>
        where V : IEquatable<V>
    {
        /// <summary>
        /// Add a new leaf to the tree.
        /// </summary>
        /// <param name="parent">Parent vertex of the new leaf.</param>
        /// <param name="vertexData">Data contained in the new leaf.</param>
        public Vertex AddLeaf(Vertex parent, V data)
        {
            return base.AddLeaf(parent, data, 0);
        }

        public static Tree<V> InferFrom(ColoredTree<V, Byte> tree)
        {
            Tree<V> result = new Tree<V>();
            result.SetVertices(tree.Vertices.ToList());
            result.SetEdges(tree.Edges.ToList());
            result.IsDirected = tree.IsDirected;
            return result;
        }
    }
}
