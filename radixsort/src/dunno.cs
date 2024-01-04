using System;
using System.Linq;
using System.Text;
using AnimLib;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Threading;

public class Table {
    Shape outerBorder;
    Shape rowSeparators;
    Shape colSeparators;

    int rows, cols;
    Vector2 bottomLeft;
    Vector2 topRight;

    Color borderColor;

    float[] colsSeps = null;
    float[] rowSeps = null;

    List<(Shape s, char c)>[,] texts;
    Color[,] colors;

    Animator animator;

    public Color BorderColor {
        get {
            return outerBorder.ContourColor;
        } set {
            outerBorder.ContourColor = value;
            if(rowSeparators != null)
                rowSeparators.ContourColor = value;
            if(colSeparators != null)
                colSeparators.ContourColor = value;
        }
    }

    public float OuterWidth {
        set {
            outerBorder.ContourSize = value;
        }
    }

    public float InnerWidth {
        set {
            if(rowSeparators != null)
                rowSeparators.ContourSize = value;
            if(colSeparators != null)
                colSeparators.ContourSize = value;
        }
    }

    public Transform2D Transform {
        get {
            return outerBorder.Transform;
        }
    }

    public Table(int rows, int columns, Vector2 min, Vector2 size, Animator animator, float borderWidth = 3.0f) {
        this.animator = animator;

        var max = min + size;
        var pb = new PathBuilder();
        var center = 0.5f*(min+max);

        float width = size.x;
        float height = size.y;

        var bl = new Vector2(-0.5f*width, -0.5f*height); // bottom left
        this.bottomLeft = bl;
        this.topRight = bl + size;
        this.rows = rows;
        this.cols = columns;

        pb.MoveTo(bl);
        pb.LineTo(new Vector2(0.5f*width, -0.5f*height)); // bottom right
        pb.LineTo(new Vector2(0.5f*width, 0.5f*height)); // top right
        pb.LineTo(new Vector2(-0.5f*width, 0.5f*height));
        pb.Close();

        var shape = new Shape(pb);
        shape.Mode = ShapeMode.Contour;
        shape.ContourColor = Color.BLACK;
        shape.ContourSize = borderWidth;
        shape.Transform.Pos = center;

        texts = new List<(Shape s, char c)>[rows, cols];
        colors = new Color[rows, cols];
        for(int i = 0; i < colors.GetLength(0); i++)
        for(int j = 0; j < colors.GetLength(1); j++) {
            colors[i, j] = Color.WHITE;
        }

        this.outerBorder = shape;
    }

    static float[] DefaultRowSeparators(int rows, float height) {
        var rowSeps = new float[rows-1];
        for(int i = rows-1; i >= 1; i--) {
            float offset = height / rows;
            rowSeps[i-1] = offset*i;
        }
        return rowSeps;
    }

    static float[] DefaultColSeparators(int cols, float width) {
        var colSeps = new float[cols-1];
        for (int i = 1; i < cols; i++) {
            float offset = width / cols ;
            colSeps[i-1] = offset*i;
        }
        return colSeps;
    }

    public void SetSeparators(float[] ro, float[] co) {
        var size = topRight - bottomLeft;
        if(ro == null) ro = DefaultRowSeparators(rows, size.y);
        if(co == null) co = DefaultColSeparators(cols, size.x);
        CreateSeparators(ro, co);
    }

    void CreateSeparators(float[] rowOffsets, float[] colOffsets) {
        this.colsSeps = colOffsets;
        this.rowSeps = rowOffsets;
        if(rowOffsets.Length != rows-1 || colOffsets.Length != cols-1) {
            throw new ArgumentException();
        }
        var pb = new PathBuilder();
        var size = topRight - bottomLeft;
        foreach(var r in rowOffsets) {
            pb.MoveTo(bottomLeft + new Vector2(0.0f, r));
            pb.LineTo(bottomLeft + new Vector2(size.x, r));
        }
        rowSeparators = new Shape(pb);
        rowSeparators.Mode = ShapeMode.Contour;
        rowSeparators.ContourColor = BorderColor;
        //rowSeparators.Transform.Pos = 0.5f*(bottomLeft+topRight);
        rowSeparators.Transform.parent = outerBorder.Transform;
        pb.Clear();
        foreach(var c in colOffsets) {
            pb.MoveTo(bottomLeft + new Vector2(c, 0.0f));
            pb.LineTo(bottomLeft + new Vector2(c, size.y));
        }
        colSeparators = World.current.Clone(rowSeparators);
        colSeparators.Path = pb;
        colSeparators.Transform.parent = outerBorder.Transform;
    }

    // zero based indices
    public Vector2 GetCellPoint(int row, int col, Vector2 pivot = default) {
        var size = topRight - bottomLeft;
        float right = col == (cols-1) ? size.x : colsSeps[col];
        float left = col == 0 ? 0.0f : colsSeps[col-1];
        float w = right-left;
        float top = row == 0 ? size.y : rowSeps[row-1];
        float bottom = row == (rows-1) ? 0.0f : rowSeps[row];
        float h = top - bottom;
        return outerBorder.Transform.Pos + bottomLeft + new Vector2(0.5f*(left+right), 0.5f*(bottom+top)) + pivot*new Vector2(w, h);
    }

    public async Task CreateFadeIn(float duration) {
        if(outerBorder.created) return;
        var wait =  World.current.CreateFadeIn(outerBorder, duration);
        if(rowSeparators != null) _ = World.current.CreateFadeIn(rowSeparators, duration);
        if(colSeparators != null) _ = World.current.CreateFadeIn(colSeparators, duration);
        foreach(var t in texts) {
            if(t == null) continue;
            t.ForEach(x => _ = World.current.CreateFadeIn(x.s, duration));
        }
        await wait;
    }

    public async Task DestroyFadeOut(float duration) {
        if(!outerBorder.created) return;
        if(rowSeparators != null) _ = World.current.DestroyFadeOut(rowSeparators, duration);
        if(colSeparators != null) _ = World.current.DestroyFadeOut(colSeparators, duration);
        foreach(var t in texts) {
            if(t == null) continue;
            t.ForEach(x => _ = World.current.DestroyFadeOut(x.s, duration));
        }
        await World.current.DestroyFadeOut(outerBorder, duration);
    }

    public void SetText(int row, int col, string text, int size, string font = null) {
        if(row < 0 || row > rows-1) return;
        if(col < 0 || col > cols-1) return;
        var cellCenter = GetCellPoint(row, col, new Vector2(-0.5f, 0.0f)) - Transform.Pos + new Vector2(10.0f, -size/2);
        var textShape = animator.ShapeText(text, cellCenter, size, font);
        textShape.ForEach(x => x.s.Color = colors[row,col]);
        textShape.ForEach(x => x.s.Transform.parent = Transform);
        if(texts[row, col] != null && outerBorder.created) {
            foreach(var t in texts[row, col]) {
                World.current.Destroy(t.s);
            }
        }
        texts[row, col] = textShape;
        if(outerBorder.created) {
            foreach(var t in textShape) {
                World.current.CreateInstantly(t.s);
            }
        }
    }

    public void SetTextColor(int row, int col, Color color) {
        if(row < 0 || row > rows-1) return;
        if(col < 0 || col > cols-1) return;
        colors[row,col] = color;
        if(texts[row, col] != null) {
            foreach(var t in texts[row, col]) {
                t.s.Color = color;
            }
        }
    }
}

public class Dunno : AnimationBehaviour
{
    public void Init(AnimationSettings settings) {
        settings.Name = "My animation";
        // animation length must be bounded
        // (it gets "baked" to allow seeking whole animation in editor)
        settings.MaxLength = 600.0f; 
    }

    const int barCount = 40;
    Vector2[,] barPositions = new Vector2[2, barCount];
    Rectangle[,] barBoxes = new Rectangle[2, barCount];

    Dictionary<Rectangle, float> values = new Dictionary<Rectangle, float>();
    //Dictionary<Rectangle, object> valueTextDummies = new Dictionary<Rectangle, object>();
    Dictionary<Rectangle, object> valueTexts = new Dictionary<Rectangle, object>();

    World world;
    Animator animator;

    Random srand = new Random();

    public async Task CopyTo(Rectangle[] src, Rectangle[] dst, int srcIdx, int dstIndex, bool flipped, double duration = 1.0) {
        if(dst[dstIndex] != null) {
            _ = world.DestroyFadeOut(dst[dstIndex], (float)(duration/2.0));
            var temp = dst[dstIndex];
            dst[dstIndex] = null;
        }
        var clone = world.CreateClone(src[srcIdx]);
        var col = src[srcIdx].Color.ToVector4();
        float gray = 0.2126f * col.x + 0.7152f * col.y + 0.0722f * col.z;
        _ = Animate.Color(src[srcIdx], src[srcIdx].Color, new Color(gray, gray, gray, 1.0f), 0.85);
        //var textDummy = valueTextDummies[src[srcIdx]] as VisualEntity2D;
        //textDummy.Transform.parent = clone.Transform;
        //valueTextDummies[clone] = textDummy;
        valueTexts[clone] = valueTexts[src[srcIdx]];
        (valueTexts[src[srcIdx]] as VisualEntity2D).Transform.parent = clone.Transform;
        if(duration != 0.0)
            await Animate.Move(clone.Transform, barPositions[flipped ? 0 : 1, dstIndex], duration/2.0);
        else
            clone.Transform.Pos = barPositions[flipped ? 0 : 1, dstIndex];
        dst[dstIndex] = clone;
        values[clone] = values[src[srcIdx]];
    }

    public Text2D[] CreateText(string[] lines, Vector2 pos, Color color, float size = 22.0f, float spacing = 1.5f, string font = "monospace") {
        var ret = new Text2D[lines.Length];
        for(int i = 0; i < lines.Length; i++) {
            var line = new Text2D(lines[i],
                size: size,
                font: font,
                color: color
            );
            line.Transform.Pos = pos - (float)i*new Vector2(0.0f, spacing*size);
            ret[i] = line;
        }
        return ret;
    }

    /*public List<Shape>[] CreateText2(string[] lines, Vector2 pos, Color color, int size = 22, float spacing = 1.5f, string font = "monospace") {
        var ret = new List<Shape>[lines.Length];
        for(int i = 0; i < lines.Length; i++) {
            var text = animator.ShapeText(lines[i], pos + (float)i*new Vector2(0.0f, -size*spacing), size, font);
            text.ForEach(x => x.Color = color);
            ret[i] = text;
        }
        return ret;
    }*/

    string toBase4(int num) {
        var sb = new StringBuilder();
        //int digits = (int)Math.Ceiling(Math.Log(num+1, 4));
        int digits = 4;
        for(int i = 0; i < digits; i++) {
            int digit = (num >> (i*2)) & 0x3;
            sb.Append(digit.ToString());
        }
        var ret = new string(sb.ToString().Reverse().ToArray());
        return ret;
    }

    SoundSample[] paperflips = new SoundSample[3];
    SoundSample[] pitches = new SoundSample[4];

    public async Task Animation(World world, Animator animator) {
        this.animator = animator;
        animator.LoadFont("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", "monospace");
        animator.LoadFont("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", "monospaceb");
        world.ActiveCamera.ClearColor = new Color(10, 5, 20, 255);

        var d0Col = animator.GetColor("digit0");
        var d1Col = animator.GetColor("digit1");
        var d2Col = animator.GetColor("digit2");
        var d3Col = animator.GetColor("digit3");
        var digitColors = new Color[] {d0Col, d1Col, d2Col, d3Col};

        var pop1 = animator.GetSoundResource("pop1.wav");
        var whoosh1 = animator.GetSoundResource("whoosh1");
        var slideonpaper = animator.GetSoundResource("snap1");
        var wetclick = animator.GetSoundResource("wetclick");
        paperflips[0] = animator.GetSoundResource("paperflip1").Value;
        paperflips[1] = animator.GetSoundResource("paperflip2").Value;
        paperflips[2] = animator.GetSoundResource("paperflip3").Value;
        pitches[0] = animator.GetSoundResource("pitch0").Value;
        pitches[1] = animator.GetSoundResource("pitch1").Value;
        pitches[2] = animator.GetSoundResource("pitch2").Value;
        pitches[3] = animator.GetSoundResource("pitch3").Value;
        var tick1 = animator.GetSoundResource("tick1").Value;
        var swu = animator.GetSoundResource("sweepup.wav").Value;
        var swd = animator.GetSoundResource("sweepdown.wav").Value;

        var originCircle = new Circle(10.0f);
        originCircle.Color = Color.RED;
        originCircle.Transform.Pos = (0.0f, 0.0f);
        world.CreateInstantly(originCircle);

        float popVolume = 0.1f;
        float shuffleVolume = 0.05f;
        float tapVolume = 0.3f;
        float countVolume = 0.01f;

        var gray = new Color(180, 180, 180, 255);
        this.world = world;
        var hw = new Text2D("Preparation", size: 32.0f, color: gray);
        hw.Transform.Pos = new Vector2(100.0f, -50.0f);
        hw.Anchor = new Vector2(-0.5f, 0.5f);
        hw.HAlign = TextHorizontalAlignment.Center;
        hw.VAlign = TextVerticalAlignment.Center;
        world.CreateInstantly(hw);
        var hw2 = world.CreateClone(hw);
        hw2.Transform.Pos = (100.0f, -100.0f);
        hw2.Color = new Color(180, 0, 0, 255);
        hw2.Size = 24.0f;
        hw2.Text = "    Sorted array (goal)";

        await Time.WaitSeconds(1.0);

        float burn = 0.65f;
        var startColor = burn * Color.RED;
        var endColor = burn * Color.VIOLET;


        Rectangle[] bars = new Rectangle[40];

        float rectSpacing = 25.0f;
        float boxHeight = 100.0f + 200.0f;

        var rect = new Rectangle(20.0f, 50.0f);
        rect.Transform.Pos = (-780.0f, -350.0f);
        rect.Anchor = (0.0f, 0.0f);
        rect.Pivot = (0.0f, -0.5f);
        rect.Height = 0.0f;
        rect.Color = startColor;
        rect.ContourColor = gray;
        // create bottom background bars
        for(int i = 0; i < barCount; i++) {
            var temp2 = world.Clone(rect);
            temp2.Transform.Pos = rect.Transform.Pos + (i*rectSpacing, 0.0f);
            //temp2.Mode = ShapeMode.Contour;
            temp2.Height = boxHeight;
            temp2.Color = new Color(255, 255, 255, 0);
            temp2.ContourSize = 2.0f;
            barPositions[0, i] = temp2.Transform.Pos;
            barBoxes[0, i] = temp2;
            temp2.SortKey = -2;
            _ = world.CreateFadeIn(temp2, 0.5f);
            world.PlaySound(pop1.Value, popVolume);
            await Time.WaitSeconds(1.0/60.0*3);
        }
        // create color bars
        Task last = null;
        for(int i = 0; i < barCount; i++) {
            var temp = world.Clone(rect);
            bars[i] = temp;
            values.Add(temp, (float)i*5 + (new System.Random()).Next(0,4));
            temp.Transform.Pos = rect.Transform.Pos + (i*rectSpacing, 0.0f);
            temp.Height = 100.0f + values[temp];
            temp.Color = Color.LerpHSV(startColor, endColor, (float)i/40.0f);
            temp.Mode = ShapeMode.Filled;
            last = Animate.InterpF(x => {
                    temp.Height = x;
                }, 0.0f, 100.0f + values[temp], 1.5, Animate.InterpCurve.EaseInOut);
            _ = world.CreateFadeIn(temp, 0.5f);
            var createText = async () => {
                int val = (int)(values[temp]);
                var istr = val.ToString();
                var labelText = istr + (new string(' ', 4-istr.Length)) + $"({toBase4(val)})";
                var txt = new Text2D(labelText, size: 12, font: "monospaceb", color: Color.BLACK);
                txt.Transform.Pos = new Vector2(5.0f, 10.0f);
                txt.Transform.parent = temp.Transform;
                txt.Transform.Rot = MathF.PI/2.0f;
                Array.ForEach(txt.CurrentShapes, x => x.s.SortKey = 100);
                valueTexts.Add(temp, txt);
                await Time.WaitSeconds(1.5);
                _ = world.CreateFadeIn(txt, 0.5f);
            };
            _ = createText();
        }
        await Time.WaitSeconds(2.0);
        await last;
        // create second array background bars
        for(int i = 0; i < barCount; i++) {
            var temp3 = world.Clone(rect);
            temp3.Transform.Pos = rect.Transform.Pos + (i*rectSpacing, 400.0f);
            //temp3.Mode = ShapeMode.Contour;
            temp3.Color = new Color(255, 255, 255, 0);
            temp3.Height = boxHeight;
            temp3.SortKey = -2;
            temp3.ContourSize = 2.0f;
            _ = world.CreateFadeIn(temp3, 0.5f);
            barPositions[1, i] = temp3.Transform.Pos;
            barBoxes[1, i] = temp3;

            world.PlaySound(pop1.Value, popVolume);
            await Time.WaitSeconds(1.0/60.0*3);
        }

        var positions = bars.Select(x => x.Transform.Pos).ToArray();
        var rnd = new Random();
        var newBars = bars.ToArray().OrderBy(x => rnd.Next()).ToArray();
        var newPositions = newBars.Select(x => x.Transform.Pos).ToArray();

        var list = new List<int>();

        hw2.Text = "    Shuffle";
        await Time.WaitSeconds(1.0);

        world.PlaySound(whoosh1.Value, shuffleVolume);
        await Animate.InterpF(x => {
                for(int i = 0; i < barCount; i++) {
                    bars[i].Transform.Pos = Vector3.Lerp(positions[i], newPositions[i], x);
                }
            }, 0.0f, 1.0f, 0.5, Animate.InterpCurve.EaseInOut);

        await Time.WaitSeconds(1.0);

        newBars = newBars.OrderBy(x => x.Transform.Pos.x).ToArray();

        var pointer = new Rectangle(10.0f, 40.0f);
        pointer.Color = Color.RED;
        pointer.Anchor = (-0.0f, 0.0f);
        pointer.Pivot = (0.0f, 0.5f);
        pointer.Transform.Pos = (12.5f, -355.0f);

        hw.Text = "Radix sort (LSD base 4)";
        hw2.Text = "    Sort";

        var program = new string[] {
            "countingSortBase4(A[], B[], digitIdx):",
            "   indices = createArray(4)",
            "   // count",
            "   for (i = 0; i < A.Length; i += 1):",
            "       // get base 4 digit (shift bits to right and mask 2 lowest bits)",
            "       digit = (A[i] >> (digitIdx * 2)) & 0x3",
            "       indices[digit] += 1",
            "   // accumulate previous counts (calculate indices)",
            "   for (i = 1; i < 4; i += 1):",
            "       indices[i] += indices[i - 1]",
            "   // shift indices to right 1 place (first index is 0)",
            "   for (i = 3; i > 0; i -= 1):",
            "       indices[i] = indices[i - 1]",
            "   indices[0] = 0",
            "   // move elements to place reserved for digit",
            "   for (i = 0; i < A.Length; i += 1):",
            "       digit = (A[i] >> (digitIdx * 2)) & 0x3",
            "       idx = indices[digit]",
            "       B[idx] = A[i]",
            "       indices[digit] += 1",
            "",
            "radixSort(a[], b[], maxdigits):",
            "   A = a",
            "   B = b",
            "   for (i = 0; i < maxdigits; i += 1):",
            "       countingSortBase4(A, B, i)",
            "       temp = A",
            "       A = B",
            "       B = temp",
            "",
            "",
            "",
            "",
            "",
            "",
        };

        var program2 = new string[] {
            "countingSortBase4(A[], B[], digitIdx, start, end):",
            "   indices = createArray(4)",
            "   // count",
            "   for (i = start; i < end; i += 1):",
            "       // get base 4 digit (shift bits to right and mask 2 lowest bits)",
            "       digit = (A[i] >> (digitIdx * 2)) & 0x3",
            "       indices[digit] += 1",
            "   // accumulate previous counts (calculate indices)",
            "   indices[0] += start",
            "   for (i = 1; i < 4; i += 1):",
            "       indices[i] += indices[i - 1]",
            "   // shift indices to right 1 place (first index is 0)",
            "   for (i = 3; i > 0; i -= 1):",
            "       indices[i] = indices[i - 1]",
            "   indices[0] = start",
            "   // make clone of indices for return value",
            "   ret = clone(indices)",
            "   // move elements to place reserved for digit",
            "   for (i = start; i < end; i += 1):",
            "       digit = (A[i] >> (digitIdx * 2)) & 0x3",
            "       idx = indices[digit]",
            "       B[idx] = A[i]",
            "       indices[digit] += 1",
            "   return ret",
            "",
            "msdRadixSort4(a[], b[], start, end, digitIdx):",
            "   if (digitIdx < 0 || end-start <= 0):",
            "       return",
            "   indices = countingSortBase4(a, b, digitIdx, start, end)",
            "   for (i = 0; i < 4; i += 1):",
            "       subEnd = end",
            "       if (i < 3):",
            "           subEnd = indices[i+1]",
            "       msdRadixSort4(b, a, indices[i], subEnd, digitIdx-1)",
        };

        int programTextSize = 12;

        var programPos = new Vector2(300.0f, 400.0f);

        var lines = Enumerable.Range(1, program.Length).Select(x => x.ToString()).ToArray();
        var ltexts = CreateText(lines, (270.0f, 400.0f), gray, programTextSize);
        var texts = CreateText(program, (300.0f, 400.0f), gray, programTextSize);

        var texts2 = CreateText(program2, (300.0f, 400.0f), gray, programTextSize);

        /*var startCall = world.Clone(texts[texts.Length-1]);
        startCall.Size = 18.0f;
        startCall.Text = "insertionSort(A, A.Length)";
        startCall.Transform.Pos += (0.0f, 80.0f);
        var startCallLabel = world.Clone(startCall);
        startCallLabel.Transform.Pos += (-30.0f, -1.5f*startCall.Size);
        startCallLabel.Text = "Call to sort entire array:";*/

        var upperLetter = new Text2D("B", size: 48.0f, color: gray);
        upperLetter.Anchor = (-0.5f, 0.0f);
        upperLetter.Transform.Pos = (100.0f, 200.0f);
        var lowerLetter = world.Clone(upperLetter);
        lowerLetter.Text = "A";
        lowerLetter.Transform.Pos = (100.0f, -230.0f);

        var text = new Text2D("Program code", size: 28.0f, color: Color.YELLOW);
        text.Transform.Pos = (1200.0f, 100.0f);

        var text2 = world.Clone(text);
        text2.Text = "Variables";
        text2.Color = gray;
        text2.Transform.Pos = (1270.0f, 650.0f);

        
        var sCol1 = Color.YELLOW;
        var sCol2 = new Color(30, 90, 230, 255);

        var pc = new Rectangle(20.0f, 10.0f);
        pc.Anchor = (0.0f, 0.0f);
        pc.Transform.Pos = programPos;

        _ = world.CreateFadeIn(lowerLetter, 0.5f);
        _ = world.CreateFadeIn(upperLetter, 0.5f);
        var ttt = world.CreateFadeIn(text, 0.5f);

        Vector3 start11 = text.Transform.Pos;
        _ = Animate.Sine(x => text.Transform.Pos = start11 + new Vector3(x, 0.5f*x, 0.0f), 1.0f, amplitude: 10.0f);

        texts.ToList().ForEach(x => world.CreateFadeIn(x, 0.5f));
        ltexts.ToList().ForEach(x => world.CreateFadeIn(x, 0.5f));
        //_ = world.CreateFadeIn(startCall, 0.5f);
        //_ = world.CreateFadeIn(startCallLabel, 0.5f);
        await Time.WaitSeconds(0.51);

        var commentCol = new Color(0, 150, 10, 255);
        //texts[4].Color = commentCol;
        //texts[7].Color = commentCol;
        var commentTexts = new List<Text2D>();
        commentTexts.Add(texts[2]);
        commentTexts.Add(texts[4]);
        commentTexts.Add(texts[7]);
        commentTexts.Add(texts[10]);
        commentTexts.Add(texts[14]);
        commentTexts.ForEach(x => x.Color = commentCol);
        var lineEC = texts.Select(x => new ElasticColor(y => {
                x.Color = y;
                if (commentTexts.Contains(x)) {
                    x.Color = commentCol;
                }
            }, gray, 1.0f, ElasticColor.Mode.EaseOut)).ToArray();
        lineEC.ToList().ForEach(x => x.Color = x.Color.WithA(240));


        await ttt;
        await Time.WaitSeconds(0.5);


        await Time.WaitSeconds(2.0);

        bool speed = false;
        bool turbo = false;

        var setLine = async (int x, bool wait) => {
            pc.Transform.Pos = programPos + (-50.0f, -(x-1)*1.5f*programTextSize+5.0f);
            for(int i = 0; i < texts.Length; i++) {
                if(i == x-1) {
                    lineEC[i].Color = Color.RED;
                }
            }
            if(speed | turbo) return;
            if(wait) await Time.WaitSeconds(0.1);
        };
        await setLine(22, true);
        _ = world.CreateFadeIn(pc, 0.5f);
        await Time.WaitSeconds(2.0);

        var sorting = async (CancellationToken token) => {
            int i = 0;
            while(!token.IsCancellationRequested) {
                hw2.Text = "    Sorting" + new string('.', i%4);
                await Time.WaitSeconds(0.5);
                i++;
            }
        };
        var tsrc = new CancellationTokenSource();
        var textTask = sorting(tsrc.Token);

        var arrB = new Rectangle[newBars.Length];

        var comparePointer = world.Clone(pointer);
        comparePointer.Transform.Pos += (0.0f, -100.0f);
        var comparePointer2 = world.Clone(comparePointer);
        comparePointer2.Color = Color.ORANGE;
        comparePointer2.Transform.Pos += (70.0f, 0.0f);
        world.CreateInstantly(comparePointer2);
        world.CreateInstantly(comparePointer);
        var compareText = new Text2D("≤", size: 32.0f, color: gray);
        compareText.Transform.Pos = (225.0f, 995.0f);
        world.CreateInstantly(compareText);
        compareText.Color = compareText.Color.WithA(0);
        comparePointer.ContourColor = comparePointer.ContourColor.WithA(0);
        comparePointer2.ContourColor = comparePointer2.ContourColor.WithA(0);
        comparePointer.Color = comparePointer.Color.WithA(0);
        comparePointer2.Color = comparePointer2.Color.WithA(0);

        var eA = new ElasticFloat(x => {
                compareText.Color = compareText.Color.WithA((byte)(x*255.0f));
            }, 0.0f, 2.0f, ElasticFloat.Mode.EaseInOut);
        var eB = new ElasticFloat(x => {
                comparePointer.Color = comparePointer.Color.WithA((byte)(x*255.0f));
                comparePointer2.Color = comparePointer2.Color.WithA((byte)(x*255.0f));
            }, 0.0f, 2.0f, ElasticFloat.Mode.EaseInOut);


        var showCompare = (bool leq, string orText) => {
            if(speed) return;
            if(orText == null) {
                eA.Value = 1.0f;
                eB.Value = 1.0f;
                if(leq) {
                    compareText.Text = "≤";
                } else {
                    compareText.Text = ">";
                }
            } else {
                eA.Value = 1.0f;
                compareText.Text = orText;
            }
        };

        var table = new Table(2, 5, new Vector2(300.0f, -350.0f), new Vector2(300.0f, 100.0f), animator, 3.0f);
        table.BorderColor = 0.7f*Color.WHITE;
        table.SetSeparators(null, new float[] {100.0f, 150.0f, 200.0f, 250.0f});
        var ttpos = table.Transform.Pos;
        table.SetText(0, 0, "Digit", 18);
        table.SetTextColor(0, 0, Color.WHITE);
        var cols = new Color[] {d0Col, d1Col, d2Col, d3Col};
        for (int i = 0; i < 4; i++) {
            table.SetText(0, i+1, i.ToString(), 18);
            table.SetTextColor(0, i+1, cols[i]);
        }
        table.SetText(1, 0, "Count", 18);
        for (int i = 0; i <= 4; i++) {
            table.SetTextColor(1, i, Color.WHITE);
            if (i > 0) table.SetText(1, i, "0", 18);
        }
        await table.CreateFadeIn(0.5f);

        var p1 = world.Clone(pointer);
        p1.Transform.Pos = barPositions[0, 1] - (0.0f, 20.0f);
        var p2 = world.Clone(pointer);
        p2.Transform.Pos = barPositions[0, 0] + (0.0f, -40.0f);
        p2.Color = Color.ORANGE;
        var p3 = world.Clone(pointer);
        p3.Transform.Pos = barPositions[0, 2];
        p3.Color = Color.GREEN;

        var destPtrs = new Rectangle[4];
        for(int i = 0; i < 4; i++) {
            var ptr = world.Clone(pointer);
            ptr.Color = digitColors[i];
            ptr.Transform.Pos = (-1000.0f, -1000.0f);
            world.CreateInstantly(ptr);
            destPtrs[i] = ptr;
        }

        var callTextPos = new Vector2(300.0f, -435.0f);
        var callText = animator.ShapeText("// initial call for this animation", callTextPos, 14, "monospace");
        var callText2 = animator.ShapeText("radixSort(A, B, 4)", callTextPos - (0.0f, 1*1.5f*14), 14, "monospace");
        var callText3 = animator.ShapeText("// A will have sorted array if maxdigits is even, B otherwise", callTextPos - (0.0f, 2*1.5f*14), 14, "monospace");
        callText.ForEach(x => x.s.Color = gray);
        callText.ForEach(x => world.CreateFadeIn(x.s, 0.5f));
        callText2.ForEach(x => x.s.Color = gray);
        callText2.ForEach(x => world.CreateFadeIn(x.s, 0.5f));
        callText3.ForEach(x => x.s.Color = gray);
        callText3.ForEach(x => world.CreateFadeIn(x.s, 0.5f));
        var callText2Msd = animator.ShapeText("msdRadixSort4(A, B, 0, A.Length, 3)", callTextPos - (0.0f, 1*1.5f*14), 14, "monospace");
        var callText3Msd = animator.ShapeText("// A will have sorted array if digitIdx is odd, B otherwise", callTextPos - (0.0f, 2*1.5f*14), 14, "monospace");
        callText2Msd.ForEach(x => x.s.Color = gray);
        callText3Msd.ForEach(x => x.s.Color = gray);

        var p1c = new ElasticColor(x => {
                p1.Color = x;
            }, p1.Color, 2.5f);
        var p2c = new ElasticColor(x => {
                p2.Color = x;
            }, p2.Color, 2.5f);

        _ = world.CreateFadeIn(p1, 0.5f);
        //_ = world.CreateFadeIn(p2, 0.5f);

        await Time.WaitSeconds(1.0);

        //_ = world.CreateFadeIn(p3, 0.5f);

        double pointerMoveDuration = speed ? 0.05 : 0.10;

        var stkCol = Color.WHITE;

        bool lsd = true;

        var countingSort = async (Rectangle[] A, Rectangle[] B, Func<Rectangle,int> getDigit, int buckets, int idx, int count, bool flipped) => {
            await setLine(1, !speed);
            await setLine(2, !speed);
            var indices = new int[buckets];
            var counts = new int[buckets];
            var counts2 = new int[buckets];

            var dummy1 = async (bool ret) => {
                await setLine(lsd ? 4 : 4, speed);
                if(!speed && !turbo) await Time.WaitSeconds(0.01);
                return ret;
            };
            var dummy2 = async (bool ret) => {
                await setLine(lsd ? 9 : 10, !speed);
                if(!speed && !turbo) await Time.WaitSeconds(0.1);
                return ret;
            };
            var dummy3 = async (bool ret) => {
                await setLine(lsd ? 12 : 13, !speed);
                if(!speed && !turbo) await Time.WaitSeconds(0.1);
                return ret;
            };
            var dummy4 = async (bool ret) => {
                await setLine(lsd ? 16 : 19, !speed);
                if(!speed && !turbo) await Time.WaitSeconds(0.1);
                return ret;
            };

            for(int i = 0; i < 4; i++) {
                table.SetText(1, 0, "Count", 18);
                table.SetText(1, 1+i, "0", 18);
                table.SetTextColor(1, 1+i, digitColors[i]);
            }
            await Time.WaitSeconds(speed ? 0.05 : 0.5);
            // count
            for(int i = idx; i < idx+count && await dummy1(true); i++) {
                if(!speed) await Time.WaitSeconds(0.1);
                await setLine(6, !speed);
                int digit = getDigit(A[i]);
                var wvt = setLine(7, !speed);
                world.PlaySound(tick1, countVolume);
                await Animate.Move(p1.Transform, barPositions[flipped ? 1 : 0, i], pointerMoveDuration);
                await wvt;
                indices[digit]++;
                table.SetText(1, 1+digit, indices[digit].ToString(), 18);
            }
            await Time.WaitSeconds(speed ? 0.05 : 0.5);
            indices.CopyTo(counts2, 0);
            if(!lsd) {
                await setLine(9, !speed);
                await Animate.Move(p1.Transform, table.GetCellPoint(1, 1) + new Vector2(0.0f, -40.0f), pointerMoveDuration);
            }
            indices[0] += idx;
            table.SetText(1, 0, "Index", 18);
            table.SetText(1, 1, indices[0].ToString(), 18);
            // calc index step 1
            for(int i = 1; i < indices.Length && await dummy2(true); i++) {
                await Animate.Move(p1.Transform, table.GetCellPoint(1, i+1) + new Vector2(0.0f, -40.0f), pointerMoveDuration);
                if(!speed) await Time.WaitSeconds(0.1);
                await setLine(lsd ? 10 : 11, !speed);
                indices[i] += indices[i-1];
                table.SetText(1, 1+i, indices[i].ToString(), 18);
            }
            // calc index step 2
            for(int i = indices.Length-1; i > 0 && await dummy3(true); i--) {
                await Animate.Move(p1.Transform, table.GetCellPoint(1, i+1) + new Vector2(0.0f, -40.0f), pointerMoveDuration);
                if(!speed) await Time.WaitSeconds(0.1);
                await setLine(lsd ? 13 : 14, !speed);
                indices[i] = indices[i-1];
                table.SetText(1, 1+i, indices[i].ToString(), 18);
            }
            await setLine(lsd ? 14 : 15, !speed);
            indices[0] = idx;
            await Animate.Move(p1.Transform, table.GetCellPoint(1, 1) + new Vector2(0.0f, -40.0f), pointerMoveDuration);
            if(!lsd) await setLine(lsd ? 16 : 17, !speed);
            indices.CopyTo(counts, 0);
            table.SetText(1, 1+0, indices[0].ToString(), 18);
            // paint destination bar boxes
            for(int k = idx; k < idx+count; k++) {
                var bar = barBoxes[flipped ? 0 : 1, k];
                var col = bar.ContourColor;
                for(int c = 0; c < 4; c++) {
                    if(k >= indices[c] && counts2[c] > 0 && k < ((c<3) ? indices[c+1] : 40)) {
                        _ = Animate.InterpF(x => {
                                bar.ContourColor = Color.Lerp(col, digitColors[c], x);
                            }, 0.0f, 1.0f, 0.5);
                        break;
                    }
                }
            }
            Task moveDst = Task.CompletedTask;
            // index pointers
            if(counts2[0] > 0)
                moveDst = Animate.Move(destPtrs[0].Transform, barPositions[flipped ? 0 : 1, Math.Clamp(indices[0], 0, 39)], pointerMoveDuration);
            if(counts2[1] > 0)
                moveDst = Animate.Move(destPtrs[1].Transform, barPositions[flipped ? 0 : 1, Math.Clamp(indices[1], 0, 39)], pointerMoveDuration);
            if(counts2[2] > 0)
                moveDst = Animate.Move(destPtrs[2].Transform, barPositions[flipped ? 0 : 1, Math.Clamp(indices[2], 0, 39)], pointerMoveDuration);
            if(counts2[3] > 0)
                moveDst = Animate.Move(destPtrs[3].Transform, barPositions[flipped ? 0 : 1, Math.Clamp(indices[3], 0, 39)], pointerMoveDuration);
            await Time.WaitSeconds(speed ? 0.05 : 0.5);
            // move elements
            for(int i = idx; i < idx+count && await dummy4(true); i++) {
                if(!speed) await Time.WaitSeconds(0.1);
                await Animate.Move(p1.Transform, barPositions[flipped ? 1 : 0, i], pointerMoveDuration);
                await setLine(lsd?17:20, !speed);
                var digit = getDigit(A[i]);
                await setLine(lsd?18:21, !speed);
                var idx2 = indices[digit];
                await setLine(lsd?19:22, !speed);
                world.PlaySound(pitches[digit], tapVolume);
                await CopyTo(A, B, i, idx2, flipped, 0.4);
                await setLine(lsd?20:23, !speed);
                indices[digit]++;
                //await setLine(lsd?21:23, !speed);
                table.SetText(1, 1+digit, indices[digit].ToString(), 18);
                moveDst = Animate.Move(destPtrs[digit].Transform, barPositions[flipped ? 0 : 1, Math.Clamp(indices[digit], 0, 39)], pointerMoveDuration);
            }
            await moveDst;
            foreach(var dp in destPtrs) {
                _ = Animate.Move(dp.Transform, new Vector2(-1000.0f, -1000.0f), 0.5);
            }
            if(!lsd) await setLine(24, !speed);
            return counts;
        };

        var dummyFun = async () => {
            await setLine(25, !speed);
            await Time.WaitSeconds(0.1);
            return true;
        };

        var radix4Sort = async (Rectangle[] a, Rectangle[] b) => {
            await setLine(23, !speed);
            var A = a;
            await setLine(24, !speed);
            var B = b;
            //int digitCount = (int)Math.Ceiling(Math.Log(barCount, 4));
            int digitCount = 4;
            for(int i = 0; i < digitCount && await dummyFun(); i++) {
                if(i == 1) speed = true;
                var digit4 = (Rectangle r) => {
                    var val = (int)Math.Round(values[r]);
                    var digit = (val&(3<<(i*2)))>>(i*2);
                    return digit;
                };
                foreach(var vt in valueTexts) {
                    var bcol = digitColors[digit4(vt.Key)];
                    var arr = (vt.Value as Text2D).CurrentShapes;
                    arr[8-i].s.Color = bcol;
                    arr[8-i].s.ContourColor = 0.8f*Color.WHITE;
                    arr[8-i].s.Mode = ShapeMode.FilledContour;
                    if(i != 0) {
                        arr[9-i].s.Color = Color.BLACK;
                        arr[9-i].s.Mode = ShapeMode.Filled;
                    }
                }
                for(int j = 0; j < barCount; j++) {
                    var bcol = digitColors[digit4(A[j])];
                    var sub = A == newBars ? 0 : 1;
                    var b1 = barBoxes[sub, j];
                    var b2 = barBoxes[sub == 0 ? 1 : 0, j];
                    var c1 = b1.ContourColor;
                    var c2 = b2.ContourColor;
                    _ = Animate.InterpF(x => {
                            b1.ContourColor = Color.Lerp(c1, bcol, x);
                            b2.ContourColor = Color.Lerp(c2, gray, x);
                        }, 0.0f, 1.0f, 0.5);
                }
                await setLine(26, !speed);
                await countingSort(A, B, digit4, 4, 0, a.Length, A!=a);
                await setLine(27, !speed);
                await setLine(28, !speed);
                await setLine(29, !speed);
                var temp = A;
                A = B;
                B = temp;
                lowerLetter.Text = A == newBars ? "A" : "B";
                upperLetter.Text = A == newBars ? "B" : "A";
                await Time.WaitSeconds(1.0);
            }
        };


        Func<Rectangle[], Rectangle[], int, int, int, Task> msdRadixSort = null;
        msdRadixSort = async (Rectangle[] a, Rectangle[] b, int idx, int count, int digit) => {
            lowerLetter.Text = a == newBars ? "A" : "B";
            upperLetter.Text = a == newBars ? "B" : "A";
            await setLine(26, !speed);
            await setLine(27, !speed);
            if(digit < 0 || count <= 0) 
            {
                await setLine(28, !speed);
                return;
            }
            if(count <= 0) return;
            var digit4 = (Rectangle r) => {
                var val = (int)Math.Round(values[r]);
                var dig = (val&(3<<(digit*2)))>>(digit*2);
                return dig;
            };
            for(int i = idx; i < idx+count; i++) {
                var arr = (valueTexts[a[i]] as Text2D).CurrentShapes;
                arr[8-digit].s.Color = digitColors[digit4(a[i])];
                arr[8-digit].s.ContourColor = 0.8f*Color.WHITE;
                arr[8-digit].s.Mode = ShapeMode.FilledContour;
                if(digit != 3) {
                    arr[7-digit].s.Color = Color.BLACK;
                    arr[7-digit].s.Mode = ShapeMode.Filled;
                }
            }
            for(int j = 0; j < barCount; j++) {
                var bcol = digitColors[digit4(a[j])];
                var sub = a == newBars ? 0 : 1;
                var b1 = barBoxes[sub, j];
                var b2 = barBoxes[sub == 0 ? 1 : 0, j];
                var c1 = b1.ContourColor;
                var c2 = b2.ContourColor;
                int jk = j;
                _ = Animate.InterpF(x => {
                        b1.ContourColor = Color.Lerp(c1, (jk>=idx&&jk<(idx+count))?bcol:gray, x);
                        b2.ContourColor = Color.Lerp(c2, gray, x);
                    }, 0.0f, 1.0f, 0.5);
            }

            await setLine(29, !speed);
            var buckets = await countingSort(a, b, digit4, 4, idx, count, a!=newBars);

            var dummy33 = async (bool ret) => {
                await setLine(30, !speed);
                return ret;
            };

            for(int i = 0; i < 4 && await dummy33(true); i++) {
                if(i == 1) speed = true;
                await setLine(31, !speed);
                int end = idx + count;
                await setLine(32, !speed);
                if(i < 3) {
                    await setLine(33, !speed);
                    end = buckets[i+1];
                }
                await setLine(34, !speed);
                await msdRadixSort(b, a, buckets[i], end-buckets[i], digit-1);
            }
        };

        await radix4Sort(newBars, arrB);
        //speed = false;
        lsd = false;


        var hw2red = hw2.Color;
        tsrc.Cancel();
        _ = Animate.Color(hw2, hw2.Color, new Color(0, 200, 0, 255), 0.9);
        hw2.Text = "    Sorted!";
        await Time.WaitSeconds(2.0);

        foreach(var c in valueTexts.Values) {
            var ls = (c as Text2D).CurrentShapes;
            Array.ForEach(ls, x => x.s.Mode = ShapeMode.Filled);
            Array.ForEach(ls, x => x.s.Color = Color.BLACK);
        }
        foreach(var b in barBoxes) {
            var col = b.ContourColor;
            _ = Animate.InterpF(x => {
                    b.ContourColor = Color.Lerp(col, gray, x);
                }, 0.0f, 1.0f, 0.5);
        }

        hw.Text = "Preparation";
        hw2.Text = "    Shuffle";
        _ = Animate.Color(hw2, hw2.Color, hw2red, 0.9);
        await Time.WaitSeconds(2.0);

        newBars = newBars.OrderBy(x => rnd.Next()).ToArray();
        newPositions = newBars.Select(x => x.Transform.Pos).ToArray();
        bars = newBars.OrderBy(x => x.Transform.Pos.x).ToArray();

        world.PlaySound(whoosh1.Value, shuffleVolume);
        await Animate.InterpF(x => {
                for(int i = 0; i < barCount; i++) {
                    bars[i].Transform.Pos = Vector3.Lerp(positions[i], newPositions[i], x);
                }
            }, 0.0f, 1.0f, 0.5, Animate.InterpCurve.EaseInOut);

        await Time.WaitSeconds(1.0);

        newBars = newBars.OrderBy(x => x.Transform.Pos.x).ToArray();

        hw.Text = "Radix sort (MSD base 4)";
        hw2.Text = "    Sort";

        Task codeFade = null;
        foreach(var t in texts) {
            codeFade = world.DestroyFadeOut(t, 0.5f);
        }
        await codeFade;
        var commentChars = new List<Text2D>();
        commentChars.Add(texts2[2]);
        for(int i = 0; i < texts2.Length; i++) {
            var ec = lineEC[i];
            var locali = i;
            ec.Action = (Color x) => {
                var t = texts2[locali];
                t.Color = x;
                if(locali == 2 || locali == 4 || locali == 7 || locali == 11 || locali == 15 || locali == 17) {
                    t.Color = commentCol;
                }
            };
        }
        foreach(var t in texts2) {
            _ = world.CreateFadeIn(t, 0.5f);
        }

        callText2.ForEach(x => _ = world.DestroyFadeOut(x.s, 0.5f));
        callText3.ForEach(x => _ = world.DestroyFadeOut(x.s, 0.5f));
        callText2Msd.ForEach(x => _ = world.CreateFadeIn(x.s, 0.5f));
        callText3Msd.ForEach(x => _ = world.CreateFadeIn(x.s, 0.5f));

        await Time.WaitSeconds(0.5);
        lineEC.ToList().ForEach(x => x.Color = x.Color.WithA(234));
        await Time.WaitSeconds(1.5);

        tsrc = new CancellationTokenSource();
        textTask = sorting(tsrc.Token);

        await setLine(26, !speed);
        await msdRadixSort(newBars, arrB, 0, newBars.Length, 3);


        tsrc.Cancel();
        _ = Animate.Color(hw2, hw2.Color, new Color(0, 200, 0, 255), 0.9);
        hw2.Text = "    Sorted!";
        await Time.WaitSeconds(2.0);


        await Time.WaitSeconds(3.0);
    }
}
