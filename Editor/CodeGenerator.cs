// by Freya Holmér (https://github.com/FreyaHolmer/Mathfs)

using System;
using System.Collections.Generic;

namespace Freya
{

    public class CodeGenerator
    {

        private int scope = 0;
        public List<string> content = new();

        public void Append(string s) => content.Add($"{new string('\t', scope)}{s}");
        public void Comment(string s) => Append($"// {s}");
        public void Using(string s) => Append($"using {s};");
        public void Summary(string s) => Append($"/// <summary>{s}</summary>");
        public void Param(string param, string desc) => Append($"/// <param name=\"{param}\">{desc}</param>");
        public void LineBreak() => content.Add("");

        public void AppendHeader()
        {
            Comment("by Freya Holmér (https://github.com/FreyaHolmer/Mathfs)");
            Comment($"Do not manually edit - this file is generated by {nameof(MathfsCodegen)}.cs");
            LineBreak();
        }

        public CodeScope BracketScope(string s) => new(this, s, true);
        public CodeScope Scope(string s) => new(this, s, false);
        public RegionScope ScopeRegion(string s) => new(this, s);

        public readonly struct CodeScope : IDisposable
        {

            private readonly CodeGenerator gen;
            private readonly bool includeBrackets;

            public CodeScope(CodeGenerator gen, string s, bool includeBrackets = true)
            {
                this.gen = gen;
                this.includeBrackets = includeBrackets;
                gen.Append(includeBrackets ? $"{s} {{" : s);
                gen.scope++;
            }

            public void Dispose()
            {
                gen.scope--;

                if (includeBrackets)
                {
                    gen.Append("}");
                }
            }
        }

        public readonly struct RegionScope : IDisposable
        {

            private readonly CodeGenerator gen;

            public RegionScope(CodeGenerator gen, string s)
            {
                this.gen = gen;
                gen.Append($"#region {s}");
                gen.LineBreak();
            }

            public void Dispose()
            {
                gen.LineBreak();
                gen.Append("#endregion");
            }
        }
    }

}
