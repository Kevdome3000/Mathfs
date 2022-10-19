// by Freya Holmér (https://github.com/FreyaHolmer/Mathfs)

using System.Runtime.CompilerServices;
using UnityEngine;
using static Freya.Mathfs;

namespace Freya
{

    /// <summary>Core intersection test functions.
    /// Note: these are pretty esoteric, generally it's easier to use the instance methods in each shape,
    /// such as <c>myLine.Intersect(otherThing)</c></summary>
    public static partial class IntersectionTest
    {

        // internal
        private const float PARALLEL_DETERMINANT_THRESHOLD = 0.00001f;
        private const MethodImplOptions INLINE = MethodImplOptions.AggressiveInlining;

        /// <summary>Returns whether or not these infinite lines intersect, and if they do, also returns the t-value for each infinite line</summary>
        /// <param name="aOrigin">First line origin</param>
        /// <param name="aDir">First line direction (does not have to be normalized)</param>
        /// <param name="bOrigin">Second line origin</param>
        /// <param name="bDir">Second line direction (does not have to be normalized)</param>
        /// <param name="tA">The t-value along the first line, where the intersection happened</param>
        /// <param name="tB">The t-value along the second line, where the intersection happened</param>
        public static bool LinearTValues(Vector2 aOrigin, Vector2 aDir, Vector2 bOrigin, Vector2 bDir, out float tA, out float tB)
        {
            var d = Determinant(aDir, bDir);

            if (Abs(d) < PARALLEL_DETERMINANT_THRESHOLD)
            {
                tA = tB = default;

                return false;
            }

            var aToB = bOrigin - aOrigin;
            tA = Determinant(aToB, bDir) / d;
            tB = Determinant(aToB, aDir) / d;

            return true;
        }

        // based on https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
        /// <summary>Returns the intersections between an infinite line and a circle in the form of t-values along the line where the intersections lie. Or none, if there are none</summary>
        /// <param name="lineOrigin">Line origin</param>
        /// <param name="lineDir">Line direction (does not have to be normalized)</param>
        /// <param name="circleOrigin">Center or the circle</param>
        /// <param name="radius">Radius of the circle</param>
        public static ResultsMax2<float> LinearCircleTValues(Vector2 lineOrigin, Vector2 lineDir, Vector2 circleOrigin, float radius)
        {
            var circleToLineOrigin = lineOrigin - circleOrigin;
            var a = Vector2.Dot(lineDir, lineDir); // ray len sq
            var b = 2 * Vector2.Dot(circleToLineOrigin, lineDir);
            var c = Vector2.Dot(circleToLineOrigin, circleToLineOrigin) - radius.Square();
            var discriminant = b * b - 4 * a * c;

            if (discriminant > 0)
            {
                discriminant = Sqrt(discriminant);

                if (discriminant < 0.00001f) // line is tangent to the circle, one intersection
                {
                    return new ResultsMax2<float>(-b / (2 * a));
                }

                var tA = (-b + discriminant) / (2 * a);
                var tB = (-b - discriminant) / (2 * a); // line has two intersections

                return new ResultsMax2<float>(tA, tB);
            }

            return default; // line doesn't hit it at all
        }

        /// <summary>Returns whether or not two circles intersect, and the two intersection points (if they exist)</summary>
        /// <param name="aPos">The position of the first circle</param>
        /// <param name="aRadius">The radius of the first circle</param>
        /// <param name="bPos">The position of the second circle</param>
        /// <param name="bRadius">The radius of the second circle</param>
        public static ResultsMax2<Vector2> CirclesIntersectionPoints(Vector2 aPos, float aRadius, Vector2 bPos, float bRadius)
        {
            var distSq = DistanceSquared(aPos, bPos);
            var dist = Sqrt(distSq);
            var differentPosition = dist > 0.00001f;
            var maxRad = Max(aRadius, bRadius);
            var minRad = Min(aRadius, bRadius);
            var ringsTouching = Mathf.Abs(dist - maxRad) < minRad;

            if (ringsTouching && differentPosition)
            {
                var aRadSq = aRadius * aRadius;
                var bRadSq = bRadius * bRadius;
                var lateralOffset = (distSq - bRadSq + aRadSq) / (2 * dist);
                var normalOffset = 0.5f / dist * Sqrt(4 * distSq * aRadSq - (distSq - bRadSq + aRadSq).Square());
                var tangent = (bPos - aPos) / dist;
                var normal = tangent.Rotate90CCW();
                var chordCenter = aPos + tangent * lateralOffset;

                if (normalOffset < 0.00001f)
                {
                    return new ResultsMax2<Vector2>(chordCenter); // double intersection at one point
                }

                return new ResultsMax2<Vector2>( // two intersections
                    chordCenter + normal * normalOffset,
                    chordCenter - normal * normalOffset
                );
            }

            return default; // no intersections
        }

        /// <summary>Returns whether or not two circles overlap</summary>
        /// <param name="aPos">The position of the first circle</param>
        /// <param name="aRadius">The radius of the first circle</param>
        /// <param name="bPos">The position of the second circle</param>
        /// <param name="bRadius">The radius of the second circle</param>
        public static bool CirclesOverlap(Vector2 aPos, float aRadius, Vector2 bPos, float bRadius)
        {
            var dist = Vector2.Distance(aPos, bPos);
            var maxRad = Max(aRadius, bRadius);
            var minRad = Min(aRadius, bRadius);

            return Mathf.Abs(dist - maxRad) < minRad;
        }

        /// <summary>Returns whether or not a line passes through a box centered at (0,0)</summary>
        /// <param name="extents">Box extents/"radius" per axis</param>
        /// <param name="pt">A point in the line</param>
        /// <param name="dir">The direction of the line</param>
        public static bool LineRectOverlap(Vector2 extents, Vector2 pt, Vector2 dir)
        {
            var corner = new Vector2(extents.x, extents.y * -Sign(dir.x * dir.y));

            return SignAsInt(Determinant(dir, corner - pt)) != SignAsInt(Determinant(dir, -corner - pt));
        }

        /// <summary>Returns the intersection points of a line passing through a box</summary>
        /// <param name="center">Center of the box</param>
        /// <param name="extents">Box extents/"radius" per axis</param>
        /// <param name="pt">A point in the line</param>
        /// <param name="dir">The direction of the line</param>
        public static ResultsMax2<Vector2> LinearRectPoints(Vector2 center, Vector2 extents, Vector2 pt, Vector2 dir)
        {
            const float FLAT_THRESH = 0.000001f;

            // place the line relative to the box
            pt.x -= center.x;
            pt.y -= center.y;

            // Vertical line
            if (dir.x.Abs() < FLAT_THRESH)
            {
                if (pt.x.Abs() <= extents.x) // inside - two intersections
                {
                    return new ResultsMax2<Vector2>(
                        new Vector2(center.x + pt.x, center.y - extents.y),
                        new Vector2(center.x + pt.x, center.y + extents.y));
                }

                return default; // outside the box
            }

            // Horizontal line
            if (dir.y.Abs() < FLAT_THRESH)
            {
                if (pt.y.Abs() <= extents.y) // inside - two intersections
                {
                    return new ResultsMax2<Vector2>(
                        new Vector2(center.x - extents.x, center.y + pt.y),
                        new Vector2(center.x + extents.x, center.y + pt.y));
                }

                return default; // outside the box
            }

            // slope intercept form y = ax+b
            var a = dir.y / dir.x;
            var b = pt.y - pt.x * a;

            // y coords on vertical lines
            var xpy = a * extents.x + b; // x = extents.x
            var xny = -a * extents.x + b; // x = -extents.x
            // x coords on horizontal lines
            var ypx = (extents.y - b) / a; // y = extents.y
            var ynx = (-extents.y - b) / a; // y = -extents.y

            // validity checks
            var xp = Abs(xpy) <= extents.y;
            var xn = Abs(xny) <= extents.y;
            var yp = Abs(ypx) <= extents.x;
            var yn = Abs(ynx) <= extents.x;

            if ((xp || xn || yp || yn) == false)
            {
                return default; // no intersections
            }

            float ax, ay, bx, by;

            if (a > 0)
            {
                // positive slope means we group results in (x,y) and (-x,-y)
                ax = xp ? extents.x : ypx;
                ay = xp ? xpy : extents.y;
                bx = xn ? -extents.x : ynx;
                by = xn ? xny : -extents.y;
            }
            else
            {
                // negative slope means we group results in (x,-y) and (-x,y)
                ax = xp ? extents.x : ynx;
                ay = xp ? xpy : -extents.y;
                bx = xn ? -extents.x : ypx;
                by = xn ? xny : extents.y;
            }

            // if the points are very close, this means we hit a corner and we should return only one point
            if (Abs(ax - bx) + Abs(ay - by) < 0.000001f)
            {
                return new ResultsMax2<Vector2>(new Vector2(center.x + ax, center.y + ay));
            }

            // else, two points
            return new ResultsMax2<Vector2>(
                new Vector2(center.x + ax, center.y + ay),
                new Vector2(center.x + bx, center.y + by));
        }

        /// <summary>Returns whether or not two discs overlap. Unlike circles, discs overlap even if one is smaller and is completely inside the other</summary>
        /// <param name="aPos">The position of the first disc</param>
        /// <param name="aRadius">The radius of the first disc</param>
        /// <param name="bPos">The position of the second disc</param>
        /// <param name="bRadius">The radius of the second disc</param>
        [MethodImpl(INLINE)]
        public static bool DiscsOverlap(Vector2 aPos, float aRadius, Vector2 bPos, float bRadius) => DistanceSquared(aPos, bPos) <= (aRadius + bRadius).Square();


    }

}
