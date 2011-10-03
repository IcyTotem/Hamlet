using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Hamlet.Extensions
{
    public static class IEnumerableExtensions
    {
        /// <summary>
        /// Returns the index of a given item in the collection.
        /// </summary>
        /// <param name="item">The item to look for.</param>
        /// <param name="comparison">A comparison function to test whether two items of the given type are equal.</param>
        public static Int32 IndexOf<T>(this IEnumerable<T> collection, T item, Comparison<T> comparison)
        {
            Int32 index = 0;
            foreach (T element in collection)
            {
                if (comparison(element, item) == 0)
                    return index;
                index++;
            }
            return -1;
        }

        /// <summary>
        /// Returns the index of a given item in the collection.
        /// </summary>
        /// <typeparam name="T">T must implement IEquatable(Of T).</typeparam>
        /// <param name="item">The item to look for.</param>
        public static Int32 IndexOf<T>(this IEnumerable<T> collection, T item) where T : IEquatable<T>
        {
            return collection.IndexOf(item, (x, y) => x.Equals(y) ? 0 : 1);
        }

        /// <summary>
        /// Returns the index of a given item in the collection.
        /// </summary>
        /// <param name="item">The item to look for.</param>
        /// <param name="comparer">A comparer for the type T used to test whether two items are equal.</param>
        public static Int32 IndexOf<T>(this IEnumerable<T> collection, T item, IEqualityComparer<T> comparer)
        {
            return collection.IndexOf(item, (x, y) => comparer.Equals(x, y) ? 0 : 1);
        }

        public static T Min<T>(this IEnumerable<T> collection, Comparison<T> comparison)
        {
            T min = default(T);
            foreach (T item in collection)
                if (comparison(item, min) < 0)
                    min = item;
            return min;
        }

        public static T Min<T>(this IEnumerable<T> collection) where T : IComparable<T>
        {
            return Min(collection, (x, y) => x.CompareTo(y));
        }

        public static T Min<T>(this IEnumerable<T> collection, IComparer<T> comparer)
        {
            return Min(collection, (x, y) => comparer.Compare(x, y));
        }

        public static void Do<T>(this IEnumerable<T> collection, Action<T> action)
        {
            foreach (T item in collection)
                action(item);
        }
    }

    public static class ArrayCreator<T>
        where T : new()
    {
        /// <summary>
        /// Creates an array of given size filled with objects of a given type. All the objects are instantiated using the default parameterless constructor.
        /// </summary>
        /// <param name="size">Size of the array to create.</param>
        public static T[] Create(Int32 size = 0)
        {
            T[] result = new T[size];
            for (Int32 i = 0; i < size; i++)
                result[i] = new T();
            return result;
        }
    }

    public class CollectionComparer<T> : IEqualityComparer<IEnumerable<T>>
        where T : IEquatable<T>
    {
        public bool Equals(IEnumerable<T> x, IEnumerable<T> y)
        {
            IEnumerator<T> enumeratorX = x.GetEnumerator();
            IEnumerator<T> enumeratorY = y.GetEnumerator();
            Boolean movedX = false, movedY = false;

            enumeratorX.Reset();
            enumeratorY.Reset();
            while ((movedX = enumeratorX.MoveNext()) && (movedY = enumeratorY.MoveNext()))
                if (!enumeratorX.Current.Equals(enumeratorY.Current))
                    return false;

            // If movedX xor movedY == false, then one of the collections is shorter than the other,
            // therefore the two sequences are not equal
            return (movedX ^ movedY);
        }

        public int GetHashCode(IEnumerable<T> obj)
        {
            return obj.Aggregate(0, (hash, item) => hash ^ item.GetHashCode());
        }
    }
}
