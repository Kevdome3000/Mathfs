// by Freya Holmér (https://github.com/FreyaHolmer/Mathfs)

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;
using Debug = UnityEngine.Debug;

namespace Freya
{

    public static class MathfsCodegen
    {

        private class SplineType
        {
            public int degree;
            public string className;
            public string prettyName;
            public string prettyNameLower;
            public string[] paramNames;
            public string[] paramDescs;
            public string matrixName;
            public RationalMatrix4x4 charMatrix;

            public SplineType(int degree, string className, string prettyName, string matrixName, RationalMatrix4x4 charMatrix, string[] paramNames, string[] paramDescs, string[] paramDescsQuad = null)
            {
                this.degree = degree;
                this.className = className;
                this.prettyName = prettyName;
                prettyNameLower = prettyName.ToLowerInvariant();
                this.paramDescs = paramDescs;
                this.matrixName = matrixName;
                this.paramNames = paramNames;
                this.charMatrix = charMatrix;
            }

            public void AppendParamStrings(CodeGenerator gen, int degree, int i)
            {
                gen.Param(paramNames[i], paramDescs[i]);
            }
        }

        #region Type Definitions

        private static SplineType typeBezier = new(3, "Bezier", "Bézier", "cubicBezier", CharMatrix.cubicBezier,
            new[] { "p0", "p1", "p2", "p3" },
            new[]
            {
                "The starting point of the curve",
                "The second control point of the curve, sometimes called the start tangent point",
                "The third control point of the curve, sometimes called the end tangent point",
                "The end point of the curve"
            }
        );

        private static SplineType typeBezierQuad = new(2, "Bezier", "Bézier", "quadraticBezier", (RationalMatrix4x4)CharMatrix.quadraticBezier,
            new[] { "p0", "p1", "p2" },
            new[]
            {
                "The starting point of the curve",
                "The middle control point of the curve, sometimes called a tangent point",
                "The end point of the curve"
            }
        );

        private static SplineType typeHermite = new(3, "Hermite", "Hermite", "cubicHermite", CharMatrix.cubicHermite,
            new[] { "p0", "v0", "p1", "v1" },
            new[]
            {
                "The starting point of the curve",
                "The rate of change (velocity) at the start of the curve",
                "The end point of the curve",
                "The rate of change (velocity) at the end of the curve"
            }
        );

        private static SplineType typeBspline = new(3, "UBS", "B-Spline", "cubicUniformBspline", CharMatrix.cubicUniformBspline,
            new[] { "p0", "p1", "p2", "p3" },
            new[]
            {
                "The first point of the B-spline hull",
                "The second point of the B-spline hull",
                "The third point of the B-spline hull",
                "The fourth point of the B-spline hull"
            }
        );

        private static SplineType typeCatRom = new(3, "CatRom", "Catmull-Rom", "cubicCatmullRom", CharMatrix.cubicCatmullRom,
            new[] { "p0", "p1", "p2", "p3" },
            new[]
            {
                "The first control point of the catmull-rom curve. Note that this point is not included in the curve itself, and only helps to shape it",
                "The second control point, and the start of the catmull-rom curve",
                "The third control point, and the end of the catmull-rom curve",
                "The last control point of the catmull-rom curve. Note that this point is not included in the curve itself, and only helps to shape it"
            }
        );

        private static SplineType[] allSplineTypes = { typeBezier, typeBezierQuad, typeHermite, typeBspline, typeCatRom };

        #endregion

        // [MenuItem( "Assets/Port Spline Data" )]
        public static void PortSplineData()
        {
            var replacements = "";
            var replacementCount = 0;
            var gos = SceneManager.GetActiveScene().GetRootGameObjects();

            foreach (var go in gos)
            {
                foreach (var c in go.GetComponentsInChildren<Component>(true))
                {
                    var so = new SerializedObject(c); // can actually find null components??
                    so.Update();
                    var madeChanges = false;
                    var prop = so.GetIterator();

                    while (prop.Next(true))
                    {
                        if (prop.isArray == false && IsSplineType(prop.type, out var type, out var dim))
                        {
                            var ptMtx = prop.FindPropertyRelative("pointMatrix");

                            try
                            {
                                if (dim == 1)
                                {
                                    for (var i = 0; i < type.paramNames.Length; i++)
                                    {
                                        ptMtx.FindPropertyRelative($"m{i}").floatValue = prop.FindPropertyRelative(type.paramNames[i]).floatValue;
                                    }
                                }
                                else if (dim == 2)
                                {
                                    for (var i = 0; i < type.paramNames.Length; i++)
                                    {
                                        ptMtx.FindPropertyRelative($"m{i}").vector2Value = prop.FindPropertyRelative(type.paramNames[i]).vector2Value;
                                    }
                                }
                                else if (dim == 3)
                                {
                                    for (var i = 0; i < type.paramNames.Length; i++)
                                    {
                                        ptMtx.FindPropertyRelative($"m{i}").vector3Value = prop.FindPropertyRelative(type.paramNames[i]).vector3Value;
                                    }
                                }
                                else if (dim == 4)
                                {
                                    for (var i = 0; i < type.paramNames.Length; i++)
                                    {
                                        ptMtx.FindPropertyRelative($"m{i}").vector4Value = prop.FindPropertyRelative(type.paramNames[i]).vector4Value;
                                    }
                                }
                            }
                            catch
                            {
                                Debug.LogError($"Null thing in {go.name}/{c.GetType().Name}/{prop.propertyPath} of type {type.className} mtx: {type.matrixName}");
                            }

                            madeChanges = true;
                            replacements += $"Replaced: {go.name}/{c.GetType().Name}: {prop.displayName}\n";
                            replacementCount++;
                        }
                    }

                    if (madeChanges)
                    {
                        so.ApplyModifiedProperties();
                    }
                }
            }

            Debug.Log($"{replacementCount} replacements:\n{replacements}");
        }

        private static bool IsSplineType(string name, out SplineType type, out int dim)
        {
            foreach (var spline in allSplineTypes)
            {
                for (var d = 1; d <= 3; d++)
                {
                    if (name == $"{spline.className}{GetDegreeName(spline.degree, true)}{d}D")
                    {
                        dim = d;
                        type = spline;

                        return true;
                    }
                }
            }

            (type, dim) = (default, default);

            return false;
        }

        [MenuItem("Assets/Run Mathfs Codegen")]
        public static void Regenerate()
        {
            for (var dim = 1; dim < 5; dim++)
            {
                // 1D, 2D, 3D, 4D
                GenerateUniformSplineType(typeBezier, dim);
                GenerateUniformSplineType(typeBezierQuad, dim);
                GenerateUniformSplineType(typeHermite, dim);
                GenerateUniformSplineType(typeBspline, dim);
                GenerateUniformSplineType(typeCatRom, dim);
                GenerateMatrix(3, dim);
                GenerateMatrix(4, dim);
            }
        }

        public static string GetLerpName(int dim)
        {
            return dim switch
            {
                1 => "Mathfs.Lerp",
                2 => "Vector2.LerpUnclamped",
                3 => "Vector3.LerpUnclamped",
                4 => "Vector4.LerpUnclamped",
                _ => throw new IndexOutOfRangeException()
            };
        }


        private static void GenerateMatrix(int count, int dim)
        {
            const string vCompStr = "xyzw";
            const string vCompStrUp = "XYZW";
            var elemRange = Enumerable.Range(0, count).ToArray();
            var compRange = Enumerable.Range(0, dim).ToArray();
            var compRangeStr = compRange.Select(c => vCompStr[c].ToString()).ToArray();
            string JoinRange(string separator, Func<int, string> elem) => string.Join(separator, elemRange.Select(elem));
            var typePrefix = dim switch { > 1 => $"Vector{dim}", _ => "" };
            var lerpName = GetLerpName(dim);
            var elemType = dim switch { 1 => "float", > 1 => $"Vector{dim}", _ => throw new Exception("Invalid type") };
            var typeName = $"{typePrefix}Matrix{count}x1";
            var csParams = JoinRange(", ", i => $"m{i}");
            var csParamsThis = JoinRange(", ", i => $"this.m{i}");
            var ctorParams = JoinRange(", ", i => $"{elemType} m{i}");
            var indexerException = $"throw new IndexOutOfRangeException( $\"Matrix row index has to be from 0 to {count - 1}, got: {{row}}\" )";
            var indexerGetterCases = JoinRange(", ", i => $"{i} => m{i}") + $", _ => {indexerException}";
            var equalsCompare = JoinRange(" && ", i => $"m{i}.Equals( other.m{i} )");
            var equalsOpCompare = JoinRange(" && ", i => $"a.m{i} == b.m{i}");
            var lerpAtoB = JoinRange(", ", i => $"{lerpName}( a.m{i}, b.m{i}, t )");

            // generate content
            var code = new CodeGenerator();
            code.AppendHeader();
            code.Append("using System;");

            if (dim > 1) // for Vector2/3
            {
                code.Append("using UnityEngine;");
            }

            using (code.BracketScope("namespace Freya"))
            {
                code.Summary($"A {count}x1 column matrix with {elemType} values");

                using (code.BracketScope($"[Serializable] public struct {typeName}"))
                {
                    // fields
                    code.Append($"public {elemType} {csParams};");

                    // constructors
                    code.Append($"public {typeName}({ctorParams}) => ({csParamsThis}) = ({csParams});");

                    if (dim > 1)
                    {
                        // compose from float matrices
                        var s = $"public {typeName}({string.Join(", ", compRangeStr.Select(c => $"Matrix{count}x1 {c}"))}) => ";
                        s += $"({csParams}) = ({JoinRange(", ", i => $"new {elemType}({string.Join(", ", compRangeStr.Select(c => $"{c}.m{i}"))})")});";
                        code.Append(s);
                    }

                    // indexer
                    using (code.BracketScope($"public {elemType} this[int row]"))
                    {
                        code.Append($"get => row switch{{{indexerGetterCases}}};");

                        using (code.BracketScope("set"))
                        {
                            using (code.BracketScope("switch(row)"))
                            {
                                code.Append(JoinRange(" ", i => $"case {i}: m{i} = value; break;"));
                                code.Append($"default: {indexerException};");
                            }
                        }
                    }

                    // component extraction for vector-valued matrices
                    if (dim > 1)
                    {
                        for (var c = 0; c < dim; c++)
                        {
                            var cc = c;
                            var parameters = JoinRange(", ", i => $"m{i}.{vCompStr[cc]}");
                            code.Append($"public Matrix{count}x1 {vCompStrUp[c]} => new({parameters});");
                        }
                    }

                    // interpolation
                    code.Summary("Linearly interpolates between two matrices, based on a value <c>t</c>");
                    code.Param("t", "The value to blend by");
                    code.Append($"public static {typeName} Lerp( {typeName} a, {typeName} b, float t ) => new {typeName}({lerpAtoB});");

                    // comparison/operators
                    code.Append($"public static bool operator ==( {typeName} a, {typeName} b ) => {equalsOpCompare};");
                    code.Append($"public static bool operator !=( {typeName} a, {typeName} b ) => !( a == b );");
                    code.Append($"public bool Equals( {typeName} other ) => {equalsCompare};");
                    code.Append($"public override bool Equals( object obj ) => obj is {typeName} other && Equals( other );");
                    code.Append($"public override int GetHashCode() => HashCode.Combine( {csParams} );");
                }
            }

            // save/finalize
            var path = $"Assets/Mathfs/Runtime/Numerics/{typeName}.cs";
            File.WriteAllLines(path, code.content);
        }

        private static void GenerateUniformSplineType(SplineType type, int dim)
        {
            var degree = type.degree;
            var dataType = dim == 1 ? "float" : $"Vector{dim}";
            var polynomType = dim == 1 ? "Polynomial" : $"Polynomial{dim}D";
            var ptCount = degree + 1;
            var degFullLower = GetDegreeName(degree, false);
            var degShortCapital = GetDegreeName(degree, true);
            var structName = $"{type.className}{degShortCapital}{dim}D";
            var points = type.paramNames;
            var ptRange = Enumerable.Range(0, ptCount).ToArray();
            var pointDescs = type.paramDescs;
            var lerpName = GetLerpName(dim);
            var pointMatrixType = $"{(dim == 1 ? "" : dataType)}Matrix{ptCount}x1";

            string JoinRange(string separator, Func<int, string> elem) => string.Join(separator, ptRange.Select(elem));
            string JoinRangeStr(string separator, Func<string, string> elem) => string.Join(separator, points.Select(elem));

            var ctorParams = JoinRange(", ", i => $"{dataType} {type.paramNames[i]}");
            var csPoints = JoinRange(", ", i => $"{type.paramNames[i]}");

            var code = new CodeGenerator();
            code.AppendHeader();
            code.Using("System");
            code.Using("System.Runtime.CompilerServices");
            code.Using("UnityEngine");
            code.LineBreak();

            using (code.BracketScope("namespace Freya"))
            {
                code.LineBreak();

                // type definition
                code.Summary($"An optimized uniform {dim}D {degFullLower} {type.prettyNameLower} segment, with {ptCount} control points");

                using (code.BracketScope($"[Serializable] public struct {structName} : IParamSplineSegment<{polynomType},{pointMatrixType}>"))
                {
                    // intentionally always Cubic right now
                    code.LineBreak();
                    code.Append("const MethodImplOptions INLINE = MethodImplOptions.AggressiveInlining;");
                    code.LineBreak();

                    // fields
                    code.Append($"[SerializeField] {pointMatrixType} pointMatrix;");
                    code.Append($"[NonSerialized] {polynomType} curve;");
                    code.Append("[NonSerialized] bool validCoefficients;");
                    code.LineBreak();

                    // constructors
                    var ctorSummary = $"Creates a uniform {dim}D {degFullLower} {type.prettyNameLower} segment, from {ptCount} control points";
                    code.Summary(ctorSummary);

                    for (var i = 0; i < ptCount; i++)
                    {
                        type.AppendParamStrings(code, degree, i);
                    }

                    code.Append($"public {structName}( {ctorParams} ) : this(new {pointMatrixType}({csPoints})){{}}");

                    code.Summary(ctorSummary);
                    code.Param("pointMatrix", "The matrix containing the control points of this spline");
                    code.Append($"public {structName}( {pointMatrixType} pointMatrix ) => (this.pointMatrix,curve,validCoefficients) = (pointMatrix,default,false);");

                    code.LineBreak();

                    // properties
                    using (code.BracketScope($"public {polynomType} Curve"))
                    {
                        using (code.BracketScope($"get"))
                        {
                            using (code.Scope("if( validCoefficients )"))
                            {
                                code.Append("return curve; // no need to update");
                            }

                            code.Append("validCoefficients = true;");

                            using (code.Scope($"return curve = new {polynomType}("))
                            {
                                for (var icRow = 0; icRow < ptCount; icRow++)
                                {
                                    var sum = new MathSum();

                                    for (var ip = 0; ip < ptCount; ip++)
                                    {
                                        sum.AddTerm(type.charMatrix[icRow, ip], $"{type.paramNames[ip].ToUpperInvariant()}");
                                    }

                                    code.Append($"{sum}{(icRow < ptCount - 1 ? "," : "")}");
                                }
                            }

                            code.Append(");");
                        }
                        // todo: set would be possible! setting the points based on a curve
                    }

                    code.Append($"public {pointMatrixType} PointMatrix {{[MethodImpl( INLINE )] get => pointMatrix; [MethodImpl( INLINE )] set => _ = ( pointMatrix = value, validCoefficients = false ); }}");

                    for (var i = 0; i < ptCount; i++)
                    {
                        code.Summary(pointDescs[i]);
                        code.Append($"public {dataType} {points[i].ToUpperInvariant()}{{ [MethodImpl( INLINE )] get => pointMatrix.m{i}; [MethodImpl( INLINE )] set => _ = ( pointMatrix.m{i} = value, validCoefficients = false ); }}");
                    }

                    code.Summary($"Get or set a control point position by index. Valid indices from 0 to {degree}");

                    using (code.BracketScope($"public {dataType} this[ int i ]"))
                    {
                        var indexException = $"throw new ArgumentOutOfRangeException( nameof(i), $\"Index has to be in the 0 to {degree} range, and I think {{i}} is outside that range you know\" )";
                        code.Append($"get => i switch {{ {JoinRange(", ", i => $"{i} => {points[i].ToUpperInvariant()}")}, _ => {indexException} }};");
                        code.Append($"set {{ switch( i ){{ {JoinRange(" ", i => $"case {i}: {points[i].ToUpperInvariant()} = value; break;")} default: {indexException}; }}}}");
                    }

                    // equality checks
                    var compEquals = JoinRangeStr(" && ", p => $"{p.ToUpperInvariant()}.Equals( other.{p.ToUpperInvariant()} )");
                    var toStringParams = JoinRange(", ", i => $"{{pointMatrix.m{i}}}");
                    code.Append($"public static bool operator ==( {structName} a, {structName} b ) => a.pointMatrix == b.pointMatrix;");
                    code.Append($"public static bool operator !=( {structName} a, {structName} b ) => !( a == b );");
                    code.Append($"public bool Equals( {structName} other ) => {compEquals};");
                    code.Append($"public override bool Equals( object obj ) => obj is {structName} other && pointMatrix.Equals( other.pointMatrix );");
                    code.Append($"public override int GetHashCode() => pointMatrix.GetHashCode();");
                    code.Append($"public override string ToString() => $\"({toStringParams})\";");
                    code.LineBreak();

                    // typecasting
                    if (dim is 2 or 3 && degree is 3)
                    {
                        if (dim == 2)
                        {
                            // Typecast to 3D where z = 0
                            var structName3D = $"{type.className}{degShortCapital}3D";
                            code.Summary("Returns this spline segment in 3D, where z = 0");
                            code.Param("curve2D", "The 2D curve to cast to 3D");
                            var inParams = JoinRangeStr(", ", p => $"curve2D.{p.ToUpperInvariant()}");
                            code.Append($"public static explicit operator {structName3D}( {structName} curve2D ) => new {structName3D}( {inParams} );");
                        }

                        if (dim == 3)
                        {
                            // typecast to 2D where z is omitted
                            var structName2D = $"{type.className}{degShortCapital}2D";
                            code.Summary("Returns this curve flattened to 2D. Effectively setting z = 0");
                            code.Param("curve3D", "The 3D curve to flatten to the Z plane");
                            var inParams = JoinRangeStr(", ", p => $"curve3D.{p.ToUpperInvariant()}");
                            code.Append($"public static explicit operator {structName2D}( {structName} curve3D ) => new {structName2D}( {inParams} );");
                        }
                    }

                    // converting between spline types
                    if (degree == 3)
                    {
                        string[] cubicSplineTypeNames =
                        {
                            nameof(BezierCubic1D).Replace("1D", $"{dim}D"),
                            nameof(HermiteCubic1D).Replace("1D", $"{dim}D"),
                            nameof(CatRomCubic1D).Replace("1D", $"{dim}D"),
                            nameof(UBSCubic1D).Replace("1D", $"{dim}D")
                        };

                        RationalMatrix4x4[] typeMatrices =
                        {
                            CharMatrix.cubicBezier,
                            CharMatrix.cubicHermite,
                            CharMatrix.cubicCatmullRom,
                            CharMatrix.cubicUniformBspline
                        };

                        // Conversion to other cubic splines
                        for (var i = 0; i < 4; i++)
                        {
                            var targetType = cubicSplineTypeNames[i];

                            if (targetType == structName)
                            {
                                continue; // don't convert to self
                            }

                            var C = CharMatrix.GetConversionMatrix(type.charMatrix, typeMatrices[i]);

                            using (code.Scope($"public static explicit operator {targetType}( {structName} s ) =>"))
                            {
                                using (code.Scope($"new {targetType}("))
                                {
                                    for (var oPt = 0; oPt < 4; oPt++)
                                    {
                                        MathSum sum = new();

                                        for (var iPt = 0; iPt < 4; iPt++)
                                        {
                                            sum.AddTerm(C[oPt, iPt], $"s.{type.paramNames[iPt].ToUpperInvariant()}");
                                        }

                                        code.Append($"{sum}{(oPt < 3 ? "," : "")}");
                                    }
                                }

                                code.Append(");");
                            }
                        }
                    }

                    // Interpolation
                    code.Summary($"Returns a linear blend between two {type.prettyNameLower} curves");
                    code.Param("a", "The first spline segment");
                    code.Param("b", "The second spline segment");
                    code.Param("t", "A value from 0 to 1 to blend between <c>a</c> and <c>b</c>");

                    using (code.Scope($"public static {structName} Lerp( {structName} a, {structName} b, float t ) =>"))
                    {
                        using (code.Scope("new("))
                        {
                            for (var i = 0; i < ptCount; i++)
                            {
                                code.Append($"{lerpName}( a.{points[i].ToUpperInvariant()}, b.{points[i].ToUpperInvariant()}, t )" + (i == ptCount - 1 ? "" : ","));
                            }
                        }

                        code.Append(");");
                    }

                    // special case slerps for cubic beziers in 2D and 3D
                    if (dim is 2 or 3 && type == typeBezier)
                    {
                        // todo: hermite slerp
                        var slerpCast = dim == 2 ? "(Vector2)" : "";
                        code.LineBreak();
                        code.Summary($"Returns a linear blend between two {type.prettyNameLower} curves, where the tangent directions are spherically interpolated");
                        code.Param("a", "The first spline segment");
                        code.Param("b", "The second spline segment");
                        code.Param("t", "A value from 0 to 1 to blend between <c>a</c> and <c>b</c>");

                        using (code.BracketScope($"public static {structName} Slerp( {structName} a, {structName} b, float t )"))
                        {
                            code.Append($"{dataType} P0 = {lerpName}( a.P0, b.P0, t );");
                            code.Append($"{dataType} P3 = {lerpName}( a.P3, b.P3, t );");

                            using (code.Scope($"return new {structName}("))
                            {
                                code.Append($"P0,");
                                code.Append($"P0 + {slerpCast}Vector3.SlerpUnclamped( a.P1 - a.P0, b.P1 - b.P0, t ),");
                                code.Append($"P3 + {slerpCast}Vector3.SlerpUnclamped( a.P2 - a.P3, b.P2 - b.P3, t ),");
                                code.Append($"P3");
                            }

                            code.Append(");");
                        }
                    }

                    // special case splits
                    var hasSplit = type == typeBezier || type == typeBezierQuad;

                    if (hasSplit)
                    {
                        code.Summary("Splits this curve at the given t-value, into two curves that together form the exact same shape");
                        code.Param("t", "The t-value to split at");

                        using (code.BracketScope($"public ({structName} pre, {structName} post) Split( float t )"))
                        {
                            AppendBezierSplit(code, structName, dataType, degree, dim);
                        }
                    }
                }
            }

            var path = $"Assets/Mathfs/Runtime/Splines/Uniform Spline Segments/{structName}.cs";
            File.WriteAllLines(path, code.content);
        }

        private class MathSum
        {

            private Rational globalScale = Rational.One;
            private List<(Rational coeff, string var)> terms = new();

            public void AddTerm(Rational coeff, string var)
            {
                if (coeff != 0)
                {
                    terms.Add((coeff, var));
                }
            }

            private void TryOptimize()
            {
                if (terms.Count < 2)
                {
                    return; // can't optimize 0 or 1 terms
                }

                var coeff0 = terms[0].coeff.Abs();

                if (terms.TrueForAll(t => t.coeff.Abs() == coeff0))
                {
                    globalScale = coeff0;

                    for (var i = 0; i < terms.Count; i++)
                    {
                        terms[i] = (terms[i].coeff / coeff0, terms[i].var);
                    }
                }
            }

            public override string ToString()
            {
                if (terms.Count == 0)
                {
                    return "0";
                }

                TryOptimize();

                var line = "";

                for (var i = 0; i < terms.Count; i++)
                {
                    line += FormatTerm(i);
                }

                if (globalScale != 1)
                {
                    if (globalScale.n == 1)
                    {
                        line = $"({line})/{globalScale.d}";
                    }
                    else
                    {
                        line = $"{FormatRational(globalScale)}*({line})";
                    }
                }

                return line;
            }

            private string FormatRational(Rational v) => v.IsInteger ? $"{v.n}" : $"({v}f)";

            private string FormatTerm(int i)
            {
                var value = terms[i].coeff;
                var sign = i > 0 && value >= 0 ? "+" : "";
                string valueStr;
                var op = "";

                if (value == Rational.One)
                {
                    valueStr = "";
                }
                else if (value == -Rational.One)
                {
                    valueStr = "-";
                }
                else if (value > 0)
                {
                    valueStr = FormatRational(value);
                    op = "*";
                }
                else
                {
                    // value < 0
                    valueStr = FormatRational(-value);
                    sign = "-";
                    op = "*";
                }

                return $"{sign}{valueStr}{op}{terms[i].var}";
            }

        }

        public static string GetDegreeName(int d, bool shortName)
        {
            return d switch
            {
                1 => "Linear",
                2 => shortName ? "Quad" : "Quadratic",
                3 => "Cubic",
                4 => "Quartic",
                5 => "Quintic",
                _ => throw new IndexOutOfRangeException()
            };
        }

        private static readonly string[] comp = { "x", "y", "z", "w" };

        public static void AppendBezierSplit(CodeGenerator code, string structName, string dataType, int degree, int dim)
        {
            string LerpStr(string A, string B, int c) => $"{A}.{comp[c]} + ( {B}.{comp[c]} - {A}.{comp[c]} ) * t";

            void AppendLerps(string varName, string A, string B)
            {
                if (dim > 1)
                {
                    using (code.Scope($"{dataType} {varName} = new {dataType}("))
                    {
                        for (var c = 0; c < dim; c++)
                        {
                            var end = c == dim - 1 ? " );" : ",";
                            code.Append($"{LerpStr(A, B, c)}{end}");
                        }
                    }
                }
                else
                {
                    // floats
                    code.Append($"{dataType} {varName} = {A} + ( {B} - {A} ) * t;");
                }
            }

            AppendLerps("a", "P0", "P1");
            AppendLerps("b", "P1", "P2"); // this could be unrolled/optimized for the cubic case, as b is never used for the output

            if (degree == 3)
            {
                AppendLerps("c", "P2", "P3");
                AppendLerps("d", "a", "b");
                AppendLerps("e", "b", "c");
                AppendLerps("p", "d", "e");
                code.Append($"return ( new {structName}( P0, a, d, p ), new {structName}( p, e, c, P3 ) );");
            }
            else if (degree == 2)
            {
                AppendLerps("p", "a", "b");
                code.Append($"return ( new {structName}( P0, a, p ), new {structName}( p, b, P2 ) );");
            }
        }

    }

}
