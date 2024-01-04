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
    Vector2[] barPositions = new Vector2[barCount];
    Rectangle[] barBoxes = new Rectangle[barCount];

    Dictionary<Rectangle, float> values = new Dictionary<Rectangle, float>();

    World world;

    Random srand = new Random();

    public async Task Swap(Rectangle[] arr, int idx1, int idx2, double duration = 1.0) {
        if(idx1 == idx2) {
            return;
        }
        var pos1 = arr[idx1].Transform.Pos;
        var pos2 = arr[idx2].Transform.Pos;
        world.PlaySound(paperflips[srand.Next(0, paperflips.Length)], srand.NextSingle()*0.1f + 0.2f);
        await Animate.InterpF(x => {
                arr[idx1].Transform.Pos = Vector2.Lerp(pos1, pos2, x);
                arr[idx2].Transform.Pos = Vector2.Lerp(pos2, pos1, x);
            }, 0.0f, 1.0f, duration, Animate.InterpCurve.EaseInOut);
        var temp = arr[idx1];
        arr[idx1] = arr[idx2];
        arr[idx2] = temp;
    }

    public async Task CopyTo(Rectangle[] src, Rectangle[] dst, int srcIdx, int dstIndex, bool flipped, double duration = 1.0) {
        if(dst[dstIndex] != null) {
            _ = world.DestroyFadeOut(dst[dstIndex], (float)(duration/2.0));
            var temp = dst[dstIndex];
            dst[dstIndex] = null;
        }
        var clone = world.CreateClone(src[srcIdx]);
        if(duration != 0.0)
            await Animate.Move(clone.Transform, barPositions[dstIndex], duration/2.0);
        else
            clone.Transform.Pos = barPositions[dstIndex];
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

    SoundSample[] paperflips = new SoundSample[3];

    public async Task Animation(World world, Animator animator) {
        animator.LoadFont("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", "monospace");
        animator.LoadFont("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", "monospaceb");

        world.ActiveCamera.ClearColor = new Color(10, 5, 20, 255);

        var pop1 = animator.GetSoundResource("pop1.wav");
        var whoosh1 = animator.GetSoundResource("whoosh1");
        var slideonpaper = animator.GetSoundResource("snap1");
        var wetclick = animator.GetSoundResource("wetclick");
        paperflips[0] = animator.GetSoundResource("paperflip1").Value;
        paperflips[1] = animator.GetSoundResource("paperflip2").Value;
        paperflips[2] = animator.GetSoundResource("paperflip3").Value;
        var swu = animator.GetSoundResource("snapon.wav").Value;
        var swd = animator.GetSoundResource("snapoff.wav").Value;

        float popVolume = 0.1f;
        float shuffleVolume = 0.05f;
        float tapVolume = 0.3f;
        float sweepVolume = 0.05f;

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

        float rectSpacing = 25.0f;
        float boxHeight = 100.0f + 195.0f;

        var rect = new Rectangle(20.0f, 50.0f);
        rect.Transform.Pos = (0.0f, -250.0f);
        rect.Anchor = (-0.4f, 0.0f);
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
            barPositions[i] = temp2.Transform.Pos;
            barBoxes[i] = temp2;
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
            temp.Transform.Pos = rect.Transform.Pos + (i*rectSpacing, 0.0f);
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

        hw.Text = "Insertion sort";
        hw2.Text = "    Sort";

        var program = new string[] {
            "insertionSort(A[], n):",
            "   for (i = 1; i < n; i += 1):",
            "       x = A[i]",
            "       j = i - 1",
            "       // shift sorted larger elements to right",
            "       for ( ; j >= 0 && A[j] > x; j -= 1):",
            "           A[j + 1] = A[j]",
            "       // insert in place",
            "       A[j + 1] = x",
        };

        var lines = Enumerable.Range(1, program.Length).Select(x => x.ToString()).ToArray();
        var ltexts = CreateText(lines, (1220.0f, 200.0f), gray, 14.0f);
        var texts = CreateText(program, (1300.0f, 200.0f), gray, 14.0f);

        var startCall = world.Clone(texts[texts.Length-1]);
        startCall.Size = 18.0f;
        startCall.Text = "insertionSort(A, A.Length)";
        startCall.Transform.Pos += (0.0f, 80.0f);
        var startCallLabel = world.Clone(startCall);
        startCallLabel.Transform.Pos += (-30.0f, -1.5f*startCall.Size);
        startCallLabel.Text = "Call to sort entire array:";

        var lowerLetter = new Text2D();
        lowerLetter.Color = gray;
        lowerLetter.Size = 48.0f;
        lowerLetter.Text = "A";
        lowerLetter.Transform.Pos = (100.0f, 600.0f);

        var text = new Text2D();
        text.Color = gray;
        text.Size = 28.0f;
        text.Text = "Program";
        text.Transform.Pos = (1270.0f, 150.0f);

        var text2 = world.Clone(text);
        text2.Text = "Variables";
        text2.Color = gray;
        text2.Transform.Pos = (1270.0f, 650.0f);

        
        var sCol1 = Color.YELLOW;
        var sCol2 = new Color(30, 90, 230, 255);

        Vector2 pos = (1270.0f, -210.0f);
        var pc = new Rectangle(20.0f, 10.0f);
        pc.Anchor = (-0.5f, 0.5f);
        pc.Transform.Pos = pos;

        _ = world.CreateFadeIn(lowerLetter, 0.5f);
        var ttt = world.CreateFadeIn(text, 0.5f);
        texts.ToList().ForEach(x => world.CreateFadeIn(x, 0.5f));
        ltexts.ToList().ForEach(x => world.CreateFadeIn(x, 0.5f));
        _ = world.CreateFadeIn(startCall, 0.5f);
        _ = world.CreateFadeIn(startCallLabel, 0.5f);
        await Time.WaitSeconds(0.51);

        var commentCol = new Color(0, 150, 10, 255);
        texts[1].GetSubstring("i").ToList().ForEach(x => x.Color = Color.RED);
        texts[3].GetSubstring("j").ToList().ForEach(x => x.Color = Color.ORANGE);
        texts[4].Color = commentCol;
        texts[7].Color = commentCol;
        var lineEC = texts.Select(x => new ElasticColor(y => {
                x.Color = y;
                if(x == texts[1]) {
                    texts[1].GetSubstring("i").ToList().ForEach(x => x.Color = Color.RED);
                }
                if(x == texts[3]) {
                    texts[3].GetSubstring("j").ToList().ForEach(x => x.Color = Color.ORANGE);
                }
                if(x == texts[4] || x == texts[7]) {
                    texts[4].Color = commentCol;
                    texts[7].Color = commentCol;
                }
            }, gray, 1.0f, ElasticColor.Mode.EaseOut)).ToArray();
        lineEC.ToList().ForEach(x => x.Color = x.Color.WithA(240));


        await ttt;
        await Time.WaitSeconds(0.5);


        await Time.WaitSeconds(2.0);

        bool speed = false;
        bool turbo = false;

        var setLine = async (int x, bool wait) => {
            pc.Transform.Pos = pos + (0.0f, -(x-1)*1.5f*texts[0].Size);
            for(int i = 0; i < texts.Length; i++) {
                if(i == x-1) {
                    lineEC[i].Color = Color.RED;
                }
            }
            if(speed | turbo) return;
            if(wait) await Time.WaitSeconds(0.3);
        };
        await setLine(1, true);
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
        p1.Transform.Pos = barPositions[1] - (0.0f, 20.0f);
        var p2 = world.Clone(pointer);
        p2.Transform.Pos = barPositions[0] + (0.0f, -40.0f);
        p2.Color = Color.ORANGE;
        var p3 = world.Clone(pointer);
        p3.Transform.Pos = barPositions[2];
        p3.Color = Color.GREEN;

        var p1c = new ElasticColor(x => {
                p1.Color = x;
            }, p1.Color, 2.5f);
        var p2c = new ElasticColor(x => {
                p2.Color = x;
            }, p2.Color, 2.5f);

        _ = world.CreateFadeIn(p1, 0.5f);
        _ = world.CreateFadeIn(p2, 0.5f);

        await Time.WaitSeconds(1.0);

        //_ = world.CreateFadeIn(p3, 0.5f);

        double pointerMoveDuration = speed ? 0.05 : 0.15;

        var stkCol = Color.WHITE;

        var insertionSort = async (Rectangle[] A) => {
            for(int i = 1; i < A.Length; i++) {
                for (int j = i; j > 0 && values[A[j-1]] > values[A[j]]; j--) {
                    await Swap(A, j, j-1, 0.4);
                }
            }
        };

        bool retss = true;

        var cond = async () => {
            await setLine(6, speed);
            if(!speed) await Time.WaitSeconds(0.3);
            return retss;
        };

        var insertionSortOptimized = async (Rectangle[] A) => {
            await setLine(1, speed);
            for(int i = 1; i < A.Length; i++) {
                double barSpeed = speed ? 0.0 : 0.2;
                double moveSpeed = speed ? 0.15 : 0.4;
                if(i == 5) speed = true;
                if(i == 8) turbo = true;
                await setLine(2, speed);
                await Animate.Move(p1.Transform, barPositions[i] - (0.0f, 20.0f), barSpeed);
                float x = values[A[i]];
                Rectangle r = A[i];
                await setLine(3, speed);
                world.PlaySound(swd, sweepVolume);
                await Animate.Offset(r.Transform, new Vector2(0.0f, boxHeight + 20.0f), moveSpeed/2.0);
                int j = i-1;
                await setLine(4, speed);
                await Animate.Move(p2.Transform, barPositions[i-1] - (0.0f, 40.0f), barSpeed);

                for(; await cond() && j >= 0 && values[A[j]] > x; j--) {
                    await setLine(7, speed);
                    world.PlaySound(slideonpaper.Value, 0.1f);
                    await Animate.Move(A[j].Transform, barPositions[j+1], moveSpeed);
                    A[j+1] = A[j];
                    await Animate.Move(p2.Transform, barPositions[j] - (0.0f, 40.0f), barSpeed);
                }
                await setLine(9, speed);
                await Animate.Move(r.Transform, new Vector2(barPositions[j+1].x, r.Transform.Pos.y), moveSpeed);
                world.PlaySound(swu, sweepVolume);
                await Animate.Move(r.Transform, barPositions[j+1], moveSpeed/2.0);
                A[j+1] = r;
                if(speed) await Time.WaitSeconds(0.1);
            }
        };

        var Swap2 = (Rectangle[] arr, int idx1, int idx2, double duration) => {
            if(idx1 == idx2) {
                return;
            }
            var pos1 = arr[idx1].Transform.Pos;
            var pos2 = arr[idx2].Transform.Pos;
            var temp = arr[idx1];
            arr[idx1] = arr[idx2];
            arr[idx2] = temp;
        };

        var hoareRndPartition = (Rectangle[] A, int p, int r) => {
            int pivotIdx = rnd.Next(p, r);
            float pivot = values[A[pivotIdx]];
            int i = p-1;
            int j = r+1;
            while(true) {
                do {
                    i++; 
                } while(values[A[i]] < pivot);
                do {
                    j--;
                } while(values[A[j]] > pivot);
                if(i >= j)
                {
                    return j == r ? j-1 : j; // NOTE: j can be returned if pivot can't be last element
                }
                Swap2(A, i, j, 0.2);
            }
        };

        Action<Rectangle[], int, int, int> quicksort = null;
        quicksort = (Rectangle[] A, int p, int r, int level) => {
            if(r - p < 5) return;
            if(p >= 0 && r >= 0 && p < r) {
                int q = hoareRndPartition(A, p, r);
                speed = true;
                quicksort(A, p, q, level+1);
                quicksort(A, q+1, r, level+1);
            }
        };

        //await insertionSort(newBars);
        await insertionSortOptimized(newBars);

        tsrc.Cancel();

        _ = Animate.Color(hw2, hw2.Color, new Color(0, 200, 0, 255), 0.9);
        hw2.Text = "    Sorted!";

        await Time.WaitSeconds(1.0);
        hw2.Text = "    Sort sorted";
        await Time.WaitSeconds(1.0);

        await insertionSortOptimized(newBars);

        await Time.WaitSeconds(1.0);

        bars = newBars.ToArray();
        newBars = bars.ToArray().OrderBy(x => rnd.Next()).ToArray();
        quicksort(newBars, 0, newBars.Length-1, 0);
        positions = bars.Select(x => x.Transform.Pos).ToArray();
        newPositions = newBars.Select(x => x.Transform.Pos).ToArray();

        hw2.Text = "    Shuffle k-sorted";
        world.PlaySound(whoosh1.Value, shuffleVolume);
        await Animate.InterpF(x => {
                for(int i = 0; i < barCount; i++) {
                    bars[i].Transform.Pos = Vector3.Lerp(positions[i], newPositions[i], x);
                }
            }, 0.0f, 1.0f, 0.5, Animate.InterpCurve.EaseInOut);

        await Time.WaitSeconds(1.0);
        hw2.Text = "    Sort k-sorted";

        newBars = newBars.OrderBy(x => x.Transform.Pos.x).ToArray();

        await Time.WaitSeconds(1.0);
        await insertionSortOptimized(newBars);
        hw2.Text = "    Sorted!";

        await Time.WaitSeconds(3.0);
    }
}
