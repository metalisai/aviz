using System;
using System.Linq;
using AnimLib;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Threading;

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

    World world;

    public async Task CopyTo(Rectangle[] src, Rectangle[] dst, int srcIdx, int dstIndex, bool flipped, double duration = 1.0) {
        if(dst[dstIndex] != null) {
            _ = world.DestroyFadeOut(dst[dstIndex], (float)(duration/2.0));
            var temp = dst[dstIndex];
            dst[dstIndex] = null;
        }
        var clone = world.CreateClone(src[srcIdx]);
        if(duration != 0.0)
            await Animate.Move(clone.Transform, barPositions[flipped ? 0 : 1, dstIndex], duration/2.0);
        else
            clone.Transform.Pos = barPositions[flipped ? 0 : 1, dstIndex];
        dst[dstIndex] = clone;
        values[clone] = values[src[srcIdx]];
    }

    public Text2D[] CreateText(string[] lines, Vector2 pos, Color color, float size = 22.0f, float spacing = 1.5f) {
        var ret = new Text2D[lines.Length];
        for(int i = 0; i < lines.Length; i++) {
            var line = new Text2D();
            line.Transform.Pos = pos + (float)i*new Vector2(0.0f, spacing*size);
            line.Size = size;
            line.Color = color;
            line.Text = lines[i];
            ret[i] = line;
        }
        return ret;
    }

    public async Task Animation(World world, Animator animator) {
        world.ActiveCamera.ClearColor = new Color(10, 5, 20, 255);

        var pop1 = animator.GetSoundResource("pop1.wav");
        var whoosh1 = animator.GetSoundResource("whoosh1");
        var slideonpaper = animator.GetSoundResource("snap1");

        float popVolume = 0.1f;
        float shuffleVolume = 0.05f;
        float slideVolume = 0.2f;

        var gray = new Color(180, 180, 180, 255);
        this.world = world;
        var hw = new Text2D();
        hw.Transform.Pos = new Vector2(100.0f, 50.0f);
        hw.Size = 32.0f;
        hw.Color = gray;
        hw.Anchor = new Vector2(0.5f, 0.5f);
        hw.HAlign = TextHorizontalAlignment.Center;
        hw.VAlign = TextVerticalAlignment.Center;
        hw.Text = "Preparation";
        world.CreateInstantly(hw);
        var hw2 = world.CreateClone(hw);
        hw2.Transform.Pos = (100.0f, 100.0f);
        hw2.Color = new Color(180, 0, 0, 255);
        hw2.Size = 24.0f;
        hw2.Text = "    Sorted array";

        await Time.WaitSeconds(1.0);

        float burn = 0.65f;
        var startColor = burn * Color.RED;
        var endColor = burn * Color.VIOLET;


        Rectangle[] bars = new Rectangle[40];


        var rect = new Rectangle(20.0f, 50.0f);
        rect.Transform.Pos = (0.0f, -350.0f);
        rect.Anchor = (-0.4f, 0.0f);
        rect.Pivot = (0.0f, -0.5f);
        rect.Height = 0.0f;
        rect.Color = startColor;
        rect.ContourColor = gray;
        // create bottom background bars
        for(int i = 0; i < barCount; i++) {
            var temp2 = world.Clone(rect);
            temp2.Transform.Pos = rect.Transform.Pos + (i*25.0f, 0.0f);
            //temp2.Mode = ShapeMode.Contour;
            temp2.Height = 100.0f + 195.0f;
            temp2.Color = new Color(255, 255, 255, 0);
            temp2.ContourSize = 2.0f;
            barPositions[0, i] = temp2.Transform.Pos;
            barBoxes[0, i] = temp2;
            _ = world.CreateFadeIn(temp2, 0.5f);
            world.PlaySound(pop1.Value, popVolume);
            await Time.WaitSeconds(1.0/60.0*3);
        }
        // create color bars
        Task last = null;
        for(int i = 0; i < barCount; i++) {
            var temp = world.Clone(rect);
            bars[i] = temp;
            values.Add(temp, (float)i);
            temp.Transform.Pos = rect.Transform.Pos + (i*25.0f, 0.0f);
            temp.Height = 100.0f + i*5.0f;
            temp.Color = Color.LerpHSV(startColor, endColor, (float)i/40.0f);
            temp.Mode = ShapeMode.Filled;
            last = Animate.InterpF(x => {
                    temp.Height = x;
                }, 0.0f, 100.0f + i*5.0f, 1.5, Animate.InterpCurve.EaseInOut);
            _ = world.CreateFadeIn(temp, 0.5f);
        }
        await Time.WaitSeconds(2.0);
        await last;
        // create second array background bars
        for(int i = 0; i < barCount; i++) {
            var temp3 = world.Clone(rect);
            temp3.Transform.Pos = rect.Transform.Pos + (i*25.0f, 400.0f);
            //temp3.Mode = ShapeMode.Contour;
            temp3.Color = new Color(255, 255, 255, 0);
            temp3.Height = 100.0f + 195.0f;
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
        pointer.Anchor = (-0.4f, 0.0f);
        pointer.Pivot = (0.0f, 0.5f);
        pointer.Transform.Pos = (12.5f, -355.0f);

        hw.Text = "Bottom-up merge sort";
        hw2.Text = "    Sort";

        // program text
        var program = new string[] {
            "merge(A[], B[], leftIdx, rightIdx, end):",
            "   left = leftIdx, right = rightIdx, k = leftIdx",
            "   while(k < end):",
            "       if(left < rightIdx && (right >= end || A[left] <= A[right]))",
            "           B[k] = A[left]",
            "           left += 1",
            "       else:",
            "           B[k] = A[right]",
            "           right += 1",
            "       k += 1",
            "",
            "mergeSort(Arr[], count):",
            "   A = Arr",
            "   B = createEmptyArray(count)",
            "   for(w = 1; w < count; w *= 2):",
            "       for(i = 0; i < count; i += 2*w):",
            "           int rightIdx = min(i + w, count)",
            "           int end = min(i + 2*w, count)",
            "           merge(A, B, i, leftIdx, end)",
            "       // swap A and B",
            "       temp = A",
            "       A = B",
            "       B = temp",
            "   return A",
        };
        var lines = Enumerable.Range(1, 24).Select(x => x.ToString()).ToArray();
        var ltexts = CreateText(lines, (1220.0f, 200.0f), gray, 16.0f);
        var texts = CreateText(program, (1300.0f, 200.0f), gray, 16.0f);

        var upperLetter = new Text2D();
        upperLetter.Color = gray;
        upperLetter.Size = 48.0f;
        upperLetter.Text = "B";
        upperLetter.Transform.Pos = (100.0f, 300.0f);
        var lowerLetter = world.Clone(upperLetter);
        lowerLetter.Text = "A";
        lowerLetter.Transform.Pos = (100.0f, 700.0f);

        var text = new Text2D();
        text.Color = gray;
        text.Size = 28.0f;
        text.Text = "Program";
        text.Transform.Pos = (1270.0f, 150.0f);

        var text2 = world.Clone(text);
        text2.Text = "Variables";
        text2.Color = gray;
        text2.Transform.Pos = (1270.0f, 650.0f);

        _ = world.CreateFadeIn(upperLetter, 0.5f);
        _ = world.CreateFadeIn(lowerLetter, 0.5f);
        var ttt = world.CreateFadeIn(text, 0.5f);
        texts.ToList().ForEach(x => world.CreateFadeIn(x, 0.5f));
        ltexts.ToList().ForEach(x => world.CreateFadeIn(x, 0.5f));

        await ttt;
        await Time.WaitSeconds(0.5);
        
        await Time.WaitSeconds(2.0);


        Vector2 pos = (1270.0f, -210.0f);
        var pc = new Rectangle(20.0f, 10.0f);
        pc.Anchor = (-0.5f, 0.5f);
        pc.Transform.Pos = pos;
        _ = world.CreateFadeIn(pc, 0.5f);
        await Time.WaitSeconds(2.0);

        var lineEC = texts.Select(x => new ElasticColor(y => {
                x.Color = y;
                if(x == texts[1]) {
                    x.GetSubstring("left").ToList().ForEach(x => x.Color = Color.RED);
                    x.GetSubstring("right").ToList().ForEach(x => x.Color = Color.ORANGE);
                    x.GetSubstring("k").ToList().ForEach(x => x.Color = Color.GREEN);
                }
            }, gray, 1.0f, ElasticColor.Mode.EaseOut)).ToArray();

        bool speed = false;
        bool turbo = false;

        var setLine = async (int x, bool wait) => {
            pc.Transform.Pos = pos + (0.0f, -(x-1)*1.5f*16.0f);
            for(int i = 0; i < texts.Length; i++) {
                if(i == x-1) {
                    lineEC[i].Color = Color.RED;
                }
            }
            if(speed | turbo) return;
            if(wait) await Time.WaitSeconds(0.3);
        };

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
        var compareText = new Text2D();
        compareText.Size = 32.0f;
        compareText.Color = gray;
        compareText.Text = "≤";
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

        var p1 = world.Clone(pointer);
        p1.Transform.Pos = barPositions[0,0];
        var p2 = world.Clone(pointer);
        p2.Transform.Pos = barPositions[0,1] + (0.0f, -40.0f);
        p2.Color = Color.ORANGE;
        var p3 = world.Clone(pointer);
        p3.Transform.Pos = barPositions[1,0];
        p3.Color = Color.GREEN;
        _ = world.CreateFadeIn(p1, 0.5f);
        _ = world.CreateFadeIn(p2, 0.5f);
        _ = world.CreateFadeIn(p3, 0.5f);

        double pointerMoveDuration = speed ? 0.05 : 0.15;

        int width = 0;

        int mi = 0, mj = 0;
        var merge = async (Rectangle[] A, int iLeft, int iRight, int iEnd, Rectangle[] B, bool flipped) => {
            pointerMoveDuration = turbo ? 0.0 : pointerMoveDuration;
            if(iLeft < A.Length && mi != iLeft)
                await Animate.Move(p1.Transform, barPositions[flipped ? 1 : 0, iLeft], pointerMoveDuration);
            if(iRight < A.Length && mi != iRight)
                await Animate.Move(p2.Transform, barPositions[flipped ? 1 : 0, iRight] + (0.0f, -40.0f), pointerMoveDuration);
            mi = iLeft;
            mj = iRight;
            await setLine(2, !speed);
            for (int k = iLeft; k < iEnd; k++) {
                await setLine(3, !speed);
                if(width == 1 && k == 19) speed = true; // go fast after middle of first iteration
                if(width == 2 && k == 0) speed = false; // go slow at start of second iteration
                if(width == 2 && k == 9) speed = true; // go fast at quarter of second iteration
                double moveSpeed = speed ? 0.30 : 0.6;
                if(turbo) moveSpeed = 0.10;
                await setLine(4, speed);
                if(mi < iRight && (mj >= iEnd || values[A[mi]] <= values[A[mj]])) {
                    showCompare(true, mj >= iEnd ? "right ≥ end" : null);
                    var col = p1.Color;
                    if(!turbo) {
                        await Animate.Color(p1, col, Color.WHITE, speed ? 0.01 : 0.1, Animate.InterpCurve.EaseOutElastic);
                        await Animate.Color(p1, Color.WHITE, col, speed ? 0.01 : 0.1, Animate.InterpCurve.EaseOutElastic);
                    }
                    world.PlaySound(slideonpaper.Value, slideVolume);
                    await setLine(5, !speed);
                    await CopyTo(A, B, mi, k, flipped, moveSpeed);
                    mi++;
                    await setLine(6, !speed);
                    if(mi < A.Length)
                        await Animate.Move(p1.Transform, barPositions[flipped ? 1 : 0, mi], pointerMoveDuration);
                } else {
                    showCompare(false, mi < iRight ? null : "left ≥ rightIdx");
                    await setLine(7, !speed);
                    var col = p2.Color;
                    if(!turbo) {
                        await Animate.Color(p2, col, Color.WHITE, speed ? 0.01 : 0.1, Animate.InterpCurve.EaseOutElastic);
                        await Animate.Color(p2, Color.WHITE, col, speed ? 0.01 : 0.1, Animate.InterpCurve.EaseOutElastic);
                    }
                    world.PlaySound(slideonpaper.Value, slideVolume);
                    await setLine(8, !speed);
                    await CopyTo(A, B, mj, k, flipped, moveSpeed);
                    await setLine(9, !speed);
                    mj++;
                    if(mj < A.Length)
                        await Animate.Move(p2.Transform, barPositions[flipped ? 1 : 0, mj] + (0.0f, -40.0f), pointerMoveDuration);
                }
                await setLine(10, !speed);
                if(k+1 < barCount)
                    await Animate.Move(p3.Transform, barPositions[flipped ? 0 : 1, k+1], pointerMoveDuration);
            }
        };

        var mergeSort = async (Rectangle[] a, Rectangle[] b) => {
            int n = a.Length;
            await setLine(13, !speed);
            var A = a;
            await setLine(14, !speed);
            var B = b;
            for(width = 1; width < n; width *= 2)
            {
                await setLine(15, !speed);
                var lastT = Task.CompletedTask;
                for(int i = 0; i < barCount; i++) {
                    int cidx1 = i/width;
                    int cidx2 = i/(width);
                    int col = cidx1 % 2;
                    int col2 = cidx2 % 2;
                    bool flip = a != A;
                    byte pattern1 = col == 0 ? (byte) 100 : (byte) 180;
                    byte pattern2 = col2 == 0 ? (byte) 160 : (byte) 160;
                    var color1 = barBoxes[flip ? 1 : 0,i].ContourColor;
                    var color2 = barBoxes[flip ? 0 : 1,i].ContourColor;
                    var color1b = barBoxes[flip ? 1 : 0,i].Color;
                    var color2b = barBoxes[flip ? 0 : 1,i].Color;
                    int ii = i;
                    lastT = Animate.InterpF(x => {
                            Color set1, set2, set1b, set2b;
                            if(cidx2/2%2 == 0) {
                                set1 = Color.Lerp(color1, new Color(pattern1, 0, 0, 255), x);
                                set1b = Color.Lerp(color1b, new Color(pattern1, 0, 0, 30), x);
                            } else {
                                set1 = Color.Lerp(color1, new Color(0, pattern1, 0, 255), x);
                                set1b = Color.Lerp(color1b, new Color(0, pattern1, 0, 30), x);
                            }
                            if(cidx2/2%2 == 0) {
                                set2 = Color.Lerp(color2, new Color(pattern2, 0, 0, 255), x);
                                set2b = Color.Lerp(color2b, new Color(pattern2, 0, 0, 30), x);
                            } else {
                                set2 = Color.Lerp(color2, new Color(0, pattern2, 0, 255), x);
                                set2b = Color.Lerp(color2b, new Color(0, pattern2, 0, 30), x);
                            }
                            barBoxes[flip ? 1 : 0, ii].ContourColor = set1;
                            barBoxes[flip ? 0 : 1, ii].ContourColor = set2;
                            barBoxes[flip ? 1 : 0, ii].Color = set1b;
                            barBoxes[flip ? 0 : 1, ii].Color = set2b;
                        }, 0.0f, 1.0f, speed ? 0.1 : 0.5, Animate.InterpCurve.Linear);
                }
                await lastT;
                await Time.WaitSeconds(speed ? 0.15 : 2.0);

                for (int i = 0; i < n; i += 2*width)
                {
                    await setLine(16, !speed);
                    await setLine(17, !speed);
                    await setLine(18, !speed);
                    await setLine(19, !speed);
                    await merge(A, i, Math.Min(i+width, n), Math.Min(i+2*width, n), B, a != A);
                }
                var temp = A;
                await setLine(21, !speed);
                A = B;
                await setLine(22, !speed);
                B = temp;
                var t = upperLetter.Text;
                upperLetter.Text = lowerLetter.Text;
                lowerLetter.Text = t;
                await setLine(23, !speed);
                if(2*width < barCount) {
                    _ = Animate.Move(p3.Transform, barPositions[A!=a ? 0 : 1, 0], pointerMoveDuration);
                    _ = Animate.Move(p1.Transform, barPositions[A!=a ? 1 : 0, 0], pointerMoveDuration);
                    await Animate.Move(p2.Transform, barPositions[A!=a ? 1 : 0, 2*width] + (0.0f, -40.0f), pointerMoveDuration);
                }
                if(width == 4)
                    turbo = true;
            }
            await setLine(24, !speed);
        };

        await mergeSort(newBars, arrB);

        tsrc.Cancel();

        _ = Animate.Color(hw2, hw2.Color, new Color(0, 200, 0, 255), 0.9);
        hw2.Text = "    Sorted!";

        await Time.WaitSeconds(3.0);
    }
}
