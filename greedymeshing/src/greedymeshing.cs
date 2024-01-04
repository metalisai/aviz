using System;
using System.Linq;
using AnimLib;
using System.Collections.Generic;
using System.Threading.Tasks;

public class Greedymeshing : AnimationBehaviour
{
public void Init(AnimationSettings settings) {
    settings.Name = "My animation";
    // animation length must be bounded
    // (it gets "baked" to allow seeking whole animation in editor)
    settings.MaxLength = 1000.0f; 
}

const int gridSize = 8;
public bool[,,] exampleData = new bool[gridSize, gridSize, gridSize];

void initData() {
    bool[,] exData = new bool[8, 8] {
        {false,false,false,false,false,false,false,false},
        {false,true,true,true,false,true,false,true},
        {false,false,true,false,false,false,true,false},
        {false,false,true,false,false,false,true,false},
        {false,false,false,false,false,false,false,false},
        {false,false,false,false,false,true,false,false},
        {false,false,false,false,false,true,false,false},
        {false,false,false,false,false,true,true,true},
    };

    for (int x = 0; x < gridSize; x++) {
        for (int y = 0; y < gridSize; y++) {
            for (int z = 0; z < gridSize; z++) {
                exampleData[x, y, z] = exData[x, y];
            }
        }
    }
    exampleData[1, 1, 0] = false;
    exampleData[1, 1, 3] = false;
    exampleData[1, 1, 4] = false;

    exampleData[1, 4, 3] = true;
    exampleData[1, 4, 4] = true;
}

public async Task Animation(World world, Animator animator) {
    initData();

    var redCol = new Color(1.0f, 0.1f, 0.1f, 1.0f);

    var cam = world.ActiveCamera as PerspectiveCamera;

    cam.ClearColor = new Color(0.035f, 0.03f, 0.05f, 1.0f);
    //cam.ClearColor = new Color(3, 1, 4, 235);
    cam.Transform.Pos = new Vector3(-10.0f, 0.0f, -15.0f);
    //world.ActiveCamera.Transform.LookAt(new Vector3(0.0f, 0.0f, 0.0f));
    //var lookAtQuat = Quaternion.LookAt(world.ActiveCamera.Transform.Pos, new Vector3(0.0f, 0.0f, 0.0f), new Vector3(0.0f, -1.0f, 0.0f));
    //world.ActiveCamera.Transform.Rot = lookAtQuat;

    var code = new Text2D("bool data[8][8][8]");
    code.Anchor = (0.5f, 0.0f);
    code.Transform.Pos = (-400.0f, 0.0f);
    code.Color = new Color(0.2f, 0.2f, 1.0f, 1.0f);

    var lookRotation = Quaternion.LookRotation((Vector3.ZERO-cam.Transform.Pos).Normalized, Vector3.UP);
    world.ActiveCamera.Transform.Rot = lookRotation;
    //_ = Animate.OrbitAndLookAt(cam.Transform, Vector3.UP, Vector3.ZERO, 360.0f, 15.0f);
    /*_ = Animate.Update(() => {
        var toOrigin = (Vector3.ZERO - world.ActiveCamera.Transform.Pos).Normalized;
        var lookRotation = Quaternion.LookRotation(toOrigin, Vector3.UP);
        world.ActiveCamera.Transform.Rot = lookRotation;
    }, System.Threading.CancellationToken.None);*/

    var mesh = new Mesh();
    mesh.Vertices = new List<Vector3> {
        new Vector3(0.0f, 0.0f, 0.0f),
        new Vector3(1.0f, 0.0f, 0.0f),
        new Vector3(1.0f, 1.0f, 0.0f),
        new Vector3(0.0f, 1.0f, 0.0f),
    }.ToArray();
    mesh.Indices = new List<uint> {
        0, 1, 2,
        0, 2, 3,
    }.ToArray();
    mesh.Color = new Color(0.1f, 1.0f, 0.05f, 0.0f);
    mesh.Outline = new Color(1.1f, 1.1f, 0.0f, 1.0f);

    var cube = new Cube();
    cube.Color = new Color(0.1f, 0.1f, 2.0f, 0.0f);
    cube.Outline = Color.RED;
    //world.CreateInstantly(cube);

    var line = new Line3D();
    line.Color = new Color(2.0f, 0.1f, 0.1f, 1.0f);
    line.Vertices = new List<Vector3> {
        new Vector3(-3.0f, 0.0f, 0.0f),
        new Vector3(-3.0f, 5.0f, 0.0f),
    }.ToArray();
    line.Colors = new List<Color> {
        new Color(2.1f, 2.1f, 2.0f, 1.0f),
        new Color(2.1f, 0.1f, 0.1f, 1.0f),
    }.ToArray();
    line.Width = 1.0f;

    var quad = new Quad();
    quad.Color = Color.TRANSPARENT;
    quad.Outline = 3.0f*Color.BLUE;
    quad.Vertices = (
        new Vector3(0.0f, 0.0f, 0.0f), 
        new Vector3(3.0f, 0.0f, 0.0f), 
        new Vector3(3.0f, 3.0f, 0.0f), 
        new Vector3(0.0f, 3.0f, 0.0f)
    );
    quad.Transform.Pos = new Vector3(0.0f, 0.0f, 1.0f);
    //world.CreateInstantly(quad);
    //world.CreateInstantly(line);
    //world.CreateInstantly(mesh);

    Vector3 offset = new Vector3(-gridSize/2.0f - 0.5f, -gridSize/2.0f - 0.5f, -gridSize/2.0f - 0.5f);

    float gridAlpha = 0.1f;
    List<(Line3D l, int dir, (int x, int y) coord)> gridLines = new();
    Vector3 eps = new Vector3(0.001f, 0.001f, 0.001f);

    Vector3 minCorner = Vector3.ZERO + offset;

    for (int x = 0; x < gridSize+1; x++)
    for (int y = 0; y < gridSize+1; y++) {
        var line2 = new Line3D();
        line2.Color = new Color(1.0f, 0.1f, 0.1f, gridAlpha);
        line2.Vertices = new List<Vector3> {
            new Vector3(x, y, 0.0f) + offset + eps,
            new Vector3(x, y, gridSize) + offset + eps,
        }.ToArray();
        gridLines.Add((line2, 0, (x, y)));
        world.CreateInstantly(line2);

        line2 = new Line3D();
        line2.Color = new Color(0.1f, 0.1f, 1.0f, gridAlpha);
        line2.Vertices = new List<Vector3> {
            new Vector3(0, y, x) + offset + eps,
            new Vector3(gridSize, y, x) + offset+ eps,
        }.ToArray();
        gridLines.Add((line2, 1, (x, y)));
        world.CreateInstantly(line2);

        line2 = new Line3D();
        line2.Color = new Color(0.1f, 1.0f, 0.1f, gridAlpha);
        line2.Vertices = new List<Vector3> {
            new Vector3(y, 0.0f, x) + offset + eps,
            new Vector3(y, gridSize, x) + offset + eps,
        }.ToArray();
        gridLines.Add((line2, 2, (x, y)));
        world.CreateInstantly(line2);
    }

    List<VisualEntity3D> cubes = new();
    List<Vector3> targetPositions = new();

    Vector3 explodeOrign = new Vector3(-0.5f, -0.5f, -0.5f);

    for (int x = 0; x < gridSize; x++)
    for (int y = 0; y < gridSize; y++)
    for (int z = 0; z < gridSize; z++) {
        if (exampleData[x, y, z]) {
            var cube2 = new Cube();
            cube2.Color = new Color(0.1f, 0.1f, 1.0f, 1.0f);
            cube2.Outline = Color.BLACK;
            cube2.Transform.Pos = new Vector3(x, y, z) + new Vector3(-gridSize/2.0f, -gridSize/2.0f, -gridSize/2.0f);
            world.CreateInstantly(cube2);
            cubes.Add(cube2);
            targetPositions.Add(cube2.Transform.Pos);

            var toOrigin = (explodeOrign - cube2.Transform.Pos).Normalized;
            cube2.Transform.Pos -= 1000.5f*toOrigin;
        }
    }

    await Time.WaitSeconds(5.0f);
    var rexpT = new List<Task>();
    for (int i = 0; i < cubes.Count; i++) {
        rexpT.Add(Animate.Move(cubes[i].Transform, targetPositions[i], 13.0f));
    }
    async void showText() {
        await Time.WaitSeconds(4.0f);
        _ = world.CreateFadeIn(code, 3.0f);
    };
    showText();

    await Task.WhenAll(rexpT);

    rexpT.Clear();
    await Time.WaitSeconds(2.0f);
    for (int i = 0; i < cubes.Count; i++) {
        int localI = i;
        rexpT.Add(Animate.InterpF(x => cubes[localI].Transform.Scale = new Vector3(x, x, x), 1.0f, 0.2f, 1.0f));
    }
    _  = Animate.InterpF(x => code.Color = code.Color.WithA(x), 1.0f, 0.0f, 1.0f);
    await Task.WhenAll(rexpT);
    await Time.WaitSeconds(1.0f);

    Func<int, string> quadText = (int x) => {
        return "Naive quads: " + x;
    };
    var quadCountText = new Text2D(quadText(0));
    quadCountText.Color = Color.WHITE;
    quadCountText.Size = 30;
    quadCountText.Anchor = (-0.45f, 0.35f);
    world.CreateInstantly(quadCountText);
    int quadCount = 0;

    Vector3 coffset = new Vector3(-gridSize/2.0f, -gridSize/2.0f, -gridSize/2.0f);
    cube.Transform.Pos = Vector3.ZERO + coffset;
    cube.Color = Color.TRANSPARENT;
    cube.Outline = Color.TRANSPARENT;
    world.CreateInstantly(cube);
    await Animate.InterpF(x => cube.Outline = Color.LerpHSV(Color.TRANSPARENT, 1.5f*Color.RED, x), 0.0f, 1.0f, 1.0f);
    await Time.WaitSeconds(1.0f);
    var cubes2 = new List<Cube>();
    for (int i = 0; i < gridSize; i++)
    for (int j = 0; j < gridSize; j++)
    for (int k = 0; k < gridSize; k++) {
        await Animate.Move(cube.Transform, new Vector3(i, j, k) + coffset, 0.01f);
        if (exampleData[i, j, k]) {
            var cube2 = new Cube();
            cube2.Color = new Color(0.1f, 1.0f, 0.1f, 1.0f);
            cube2.Transform.Pos = new Vector3(i, j, k) + coffset;
            world.CreateInstantly(cube2);
            cubes2.Add(cube2);
            quadCountText.Text = quadText(quadCount += 6);
        }
    }
    rexpT.Clear();
    await Animate.InterpF(x => cube.Outline = Color.LerpHSV(1.5f*Color.RED, Color.TRANSPARENT, x), 0.0f, 1.0f, 1.0f);
    foreach (var cube2 in cubes2) {
        rexpT.Add(Animate.InterpF(x => {
            cube2.Transform.Scale = new Vector3(x, x, x);
            cube2.Color = Color.LerpHSV(cube2.Color, ((Cube)cubes[0]).Color, 1.0f - x);
        }, 1.0f, 0.0f, 1.0f));
    }
    await Task.WhenAll(rexpT);
    cubes2.ForEach(x => world.Destroy(x));

    Color genQuadCol = new Color(0.1f, 1.0f, 0.1f, 1.0f);
    Color genBlueCol = new Color(0.1f, 0.1f, 1.0f, 1.0f);

    await Animate.Offset(quadCountText.Transform, (0.0f, 60.0f), 1.0f);
    quadCount = 0;
    var quadCountText2 = new Text2D("Culled quads: 0");
    quadCountText2.Color = Color.WHITE;
    quadCountText2.Size = 30;
    quadCountText2.Anchor = (-0.45f, 0.35f);
    quadCountText2.Transform.Pos = (0.0f, 0.0f);
    await world.CreateFadeIn(quadCountText2, 2.0f);

    var startPos = new Vector3(-1.0f, 0.0f, 0.0f);
    cube.Transform.Pos = startPos + coffset;
    var ccube2 = world.Clone(cube);
    ccube2.Transform.Pos = startPos + Vector3.RIGHT + coffset;
    ccube2.Color = Color.TRANSPARENT;
    world.CreateInstantly(ccube2);
    await Animate.InterpF(x => {
        cube.Outline = Color.LerpHSV(Color.TRANSPARENT, 1.5f*Color.RED, x);
        ccube2.Outline = Color.LerpHSV(Color.TRANSPARENT, 1.5f*Color.BLUE, x);
    }, 0.0f, 1.0f, 1.0f);

    var dirQuad= new (Vector3, Vector3, Vector3, Vector3)[3] {
        (new Vector3(0.0f, 0.0f, 0.0f), new Vector3(0.0f, 0.0f, 1.0f), new Vector3(0.0f, 1.0f, 1.0f), new Vector3(0.0f, 1.0f, 0.0f)),
        (new Vector3(0.0f, 0.0f, 0.0f), new Vector3(0.0f, 0.0f, 1.0f), new Vector3(1.0f, 0.0f, 1.0f), new Vector3(1.0f, 0.0f, 0.0f)),
        (new Vector3(0.0f, 0.0f, 0.0f), new Vector3(0.0f, 1.0f, 0.0f), new Vector3(1.0f, 1.0f, 0.0f), new Vector3(1.0f, 0.0f, 0.0f)),
    };

    List<(Quad q, int d, int s)> quads = new();
    for (int dir = 0; dir < 3; dir++) {
        var Col2 = Color.BLUE;
        if (dir == 1) Col2 = Color.GREEN;
        else if (dir == 2) Col2 = Color.YELLOW;
        Vector3 mdir = dir switch {
            0 => Vector3.RIGHT,
            1 => Vector3.UP,
            2 => Vector3.FORWARD,
            _ => Vector3.ZERO,
        };
        startPos = dir == 0 ? new Vector3(-1.0f, 0.0f, 0.0f) : dir == 1 ? new Vector3(0.0f, -1.0f, 0.0f) : new Vector3(0.0f, 0.0f, -1.0f);
        _ = Animate.Move(cube.Transform, startPos + coffset, 0.51f);
        _ = Animate.Move(ccube2.Transform, startPos + mdir + coffset, 0.51f);
        await Animate.InterpF(x => {
            cube.Outline = Color.LerpHSV(cube.Outline, 1.5f*Color.RED, x);
            if (dir != 0)
                ccube2.Outline = Color.LerpHSV(cube.Outline, 1.5f*Col2, x);
        }, 0.0f, 1.0f, 1.0f);
        await Time.WaitSeconds(1.0f);
        
        for (int i = dir == 0 ? -1 : 0; i < gridSize; i++)
        for (int j = dir == 1 ? -1 : 0; j < gridSize; j++)
        for (int k = dir == 2 ? -1 : 0; k < gridSize; k++) {
            bool v1, v2;
            try {
                v1 = exampleData[i, j, k];
            } catch (IndexOutOfRangeException) {
                v1 = false;
            }
            (int xo, int yo, int zo) dof = (1, 0, 0);
            if (dir == 1) dof = (0, 1, 0);
            else if (dir == 2) dof = (0, 0, 1);

            try {
                v2 = exampleData[i+dof.xo, j+dof.yo, k+dof.zo];
            } catch (IndexOutOfRangeException) {
                v2 = false;
            }

            if (v1 != v2) {
                var quad2 = new Quad();
                quad2.Color = new Color(0.1f, 1.0f, 0.1f, 0.3f);
                quad2.Transform.Pos = new Vector3(i+dof.xo-0.5f, j+dof.yo-0.5f, k+dof.zo-0.5f) + coffset;
                quad2.Vertices = dirQuad[dir];
                world.CreateInstantly(quad2);
                quadCountText2.Text = "Culled quads: " + (quadCount += 1).ToString();
                int s = dir == 0 ? i : dir == 1 ? j : k;
                quads.Add((quad2, dir, s));
            }
            _ = Animate.Move(cube.Transform, new Vector3(i, j, k) + coffset, 0.01f);
            await Animate.Move(ccube2.Transform, new Vector3(i+dof.xo, j+dof.yo, k+dof.zo) + coffset, 0.01f);
        }
        await Time.WaitSeconds(1.0f);
    }

    _ = Animate.InterpF(x => {
        foreach (var quad2 in quads) {
            quad2.q.Color = quad2.q.Color.WithA(x);
        }
    }, quads[0].q.Color.A, 1.0f, 1.0f);

    await Time.WaitSeconds(2.0f);
    await Animate.InterpF(x => {
        cube.Outline = Color.LerpHSV(cube.Outline, Color.TRANSPARENT, x);
        ccube2.Outline = Color.LerpHSV(ccube2.Outline, Color.TRANSPARENT, x);
    }, 0.0f, 1.0f, 1.0f);


    await Animate.InterpF(x => {
        foreach (var quad2 in quads) {
            var targetColor = quad2.d switch {
                0 => new Color(1.0f, 0.05f, 0.05f, 1.0f),
                1 => new Color(0.05f, 1.0f, 0.05f, 1.0f),
                2 => new Color(0.05f, 0.05f, 1.0f, 1.0f),
                _ => Color.TRANSPARENT,
            };
            quad2.q.Color = Color.LerpHSV(quad2.q.Color, targetColor, x);
        }
    }, 0.0f, 1.0f, 1.0f);

    await Time.WaitSeconds(2.0f);

    // throw away yz quads
    await Animate.InterpF(x => {
        foreach (var quad in quads) {
            var dir = Vector3.ZERO - quad.q.Transform.Pos;
            if (quad.d != 0) {
                quad.q.Transform.Pos = quad.q.Transform.Pos - x*10*dir;
                quad.q.Transform.Rot = Quaternion.AngleAxis(90.0f*x, Vector3.RIGHT);
                quad.q.Color = Color.LerpHSV(quad.q.Color, Color.YELLOW.WithA(1.0f-x), x);
            }
        }
    }, 0.0f, 1.0f, 2.0f);

    await Time.WaitSeconds(2.0f);

    // burn excess slices
    await Animate.InterpF(x => {
        foreach (var quad in quads) {
            var dir = Vector3.ZERO - quad.q.Transform.Pos;
            if (quad.s != 0) {
                quad.q.Color = Color.LerpHSV(quad.q.Color, (2.0f*Color.YELLOW).WithA(1.0f-x), x);
                quad.q.Outline = quad.q.Outline.WithA(1.0f-x);
            }
        }
    }, 0.0f, 1.0f, 2.0f);

    var remainingQuads = quads.Where(x => x.s == 0 && x.d == 0).ToList();

    float deg2rad = (float)Math.PI/180.0f;
    var targetPos = new Vector3(-13.0f, 0.0f, 0.0f);
    var targetRot = Quaternion.AngleAxis(90.0f*deg2rad, Vector3.UP);
    var startRot = cam.Transform.Rot;
    var sPos = cam.Transform.Pos;
    await Animate.InterpF(x => {
        cam.Transform.Pos = Vector3.Lerp(sPos, targetPos, x);
        cam.Transform.Rot = Quaternion.Slerp(startRot, targetRot, x);
    }, 0.0f, 1.0f, 2.0f);

    // keep only layer 1 grid
    gridLines.Where(x => x.dir == 1 || (x.dir == 0 && x.coord.x != 1) || (x.dir == 2 && x.coord.y != 1)).ToList().ForEach(x => x.l.Color = x.l.Color.WithA(0.0f));

    Vector2[,] gridPositions = new Vector2[gridSize, gridSize];
    Text2D[,] gridText = new Text2D[gridSize, gridSize];

    for (int i = 0; i < 8; i++)
    for (int j = 0; j < 8; j++) {
        Vector3 ofst = coffset;
        ofst.x = 0.0f;
        Vector3 pos = ofst + new Vector3(remainingQuads[0].q.Transform.Pos.x, i, j);
        Vector2 screenPos = cam.WorldToScreenPos(pos, new Vector2(1920.0f, 1080.0f));
        gridPositions[i, j] = screenPos - new Vector2(15.0f, 22.0f/2.0f);    
    }


    int[,] slice = new int[gridSize, gridSize];
    for (int i = 0; i < gridSize; i++)
    for (int j = 0; j < gridSize; j++) {
        slice[i, j] = exampleData[0, i, j] == exampleData[1, i, j] ? 0 : exampleData[0, i, j] ? 1 : 2;
    }

    var s0 = new Text2D("0").CurrentShapes[0].s;
    var s1 = new Text2D("1").CurrentShapes[0].s;
    var s2 = new Text2D("2").CurrentShapes[0].s;
    var sliceText = new Shape[8,8];
    Vector2 ofs = new Vector2(-550.0f, -50.0f);
    for (int i = 0; i < 8; i++)
    for (int j = 0; j < 8; j++) {
        sliceText[i, j] = world.Clone(slice[j, i] > 0 ? s1 : s0);
        sliceText[i, j].Color = slice[j, i] > 0 ? redCol : Color.WHITE;
        sliceText[i, j].Transform.Pos = gridPositions[j, i];
        sliceText[i, j].Anchor = (0.0f, 0.0f);
        world.CreateInstantly(sliceText[i, j]);
    }

    List<Task> vtasks = new();
    for (int i = 0; i < 8; i++)
    for (int j = 0; j < 8; j++) {
        vtasks.Add( Animate.Move(sliceText[i, j].Transform, ofs + new Vector2(-i*40.0f, j*40.0f), 0.5f) );
    }
    await Task.WhenAll(vtasks);

    await Time.WaitSeconds(1.0f);  

    //await Animate.Offset(cam.Transform, new Vector3(0.0f, 0.0f, 2.0f), 1.0f);
    for (int i = 0; i < 8; i++)
    for (int j = 0; j < 8; j++) {
        Vector3 ofst = coffset;
        ofst.x = 0.0f;
        Vector3 pos = ofst + new Vector3(remainingQuads[0].q.Transform.Pos.x, i, j);
        Vector2 screenPos = cam.WorldToScreenPos(pos, new Vector2(1920.0f, 1080.0f));
        var nr = new Text2D($"{(i*8+j):00}");
        nr.Transform.Pos = screenPos - new Vector2(15.0f, nr.Size/2.0f);
        gridPositions[i, j] = nr.Transform.Pos;
        nr.Color = Color.WHITE;
        world.CreateInstantly(nr);
        gridText[i, j] = nr;
    }

    await Time.WaitSeconds(2.0f);

    var cursorInactiveCol = new Color(0.02f, 0.02f, 1.52f, 1.0f);
    var cursorActiveCol = Color.ORANGE*1.5f;

    Vector3 qorigin = remainingQuads[0].q.Transform.Pos - new Vector3(0.01f, 1.0f, 1.00f) - new Vector3(0.05f);
    Vector3 qorigin2 = remainingQuads[0].q.Transform.Pos - new Vector3(0.01f, 1.0f, 1.00f);
    var cursorQuad = new Quad();
    cursorQuad.Vertices = dirQuad[0];
    cursorQuad.Transform.Pos = qorigin;
    cursorQuad.Transform.Scale = new Vector3(1.1f);
    cursorQuad.Color = Color.TRANSPARENT;
    cursorQuad.Outline = cursorInactiveCol;
    world.CreateInstantly(cursorQuad);

    void unsetSliceBit(int i, int j) {
        world.Destroy(sliceText[i, j]);
        sliceText[i, j] = world.Clone(s0);
        sliceText[i, j].Color = genQuadCol;
        sliceText[i, j].Transform.Pos = ofs + new Vector2(-i*40.0f, j*40.0f);
        sliceText[i, j].Anchor = (0.0f, 0.0f);
        world.CreateInstantly(sliceText[i, j]);
    }

    List<(Quad q, int dir, int slice)> greedyQuads = new();

    void updateSlice(int dir, int sliceIdx) {
        for (int i = 0; i < gridSize; i++)
        for (int j = 0; j < gridSize; j++) {
            if (dir == 0) {
                try {
                    slice[i, j] = exampleData[sliceIdx-1, i, j] == exampleData[sliceIdx, i, j] ? 0 : exampleData[sliceIdx-1, i, j] ? 1 : 2;
                } catch (IndexOutOfRangeException) {
                    if (sliceIdx == 0) slice[i, j] = exampleData[0, i, j] ? 2 : 0;
                    else slice[i, j] = exampleData[sliceIdx-1, i, j] ? 1 : 0;
                }
            } else if (dir == 1) {
                try {
                    slice[i, j] = exampleData[i, sliceIdx-1, j] == exampleData[i, sliceIdx, j] ? 0 : exampleData[i, sliceIdx-1, j] ? 1 : 2;
                } catch (IndexOutOfRangeException) {
                    if (sliceIdx == 0) slice[i, j] = exampleData[i, 0, j] ? 2 : 0;
                    else slice[i, j] = exampleData[i, sliceIdx-1, j] ? 1 : 0;
                }
            } else if (dir == 2) {
                try {
                    slice[i, j] = exampleData[j, i, sliceIdx-1] == exampleData[j, i, sliceIdx] ? 0 : exampleData[j, i, sliceIdx-1] ? 1 : 2;
                } catch (IndexOutOfRangeException) {
                    if (sliceIdx == 0) slice[i, j] = exampleData[j, i, 0] ? 2 : 0;
                    else slice[i, j] = exampleData[j, i, sliceIdx-1] ? 1 : 0;
                }
            }
        }
    }

    async Task doSlice(int sliceIdx, int dir, bool example = false, Action onQuad = null, bool withFix = false, bool silent = false) {
        qorigin = remainingQuads[0].q.Transform.Pos 
            - new Vector3(1.001f, 1.001f, 1.001f) 
            - new Vector3(0.05f) 
            + new Vector3(dir == 0 ? sliceIdx : 0, dir == 1 ? sliceIdx : 0, dir == 2 ? sliceIdx : 0);
        qorigin2 = remainingQuads[0].q.Transform.Pos 
            - new Vector3(1.001f, 1.001f, 1.001f) 
            + new Vector3(dir == 0 ? sliceIdx : 0, dir == 1 ? sliceIdx : 0, dir == 2 ? sliceIdx : 0);

        updateSlice(dir, sliceIdx);

        Func<float, float, (Vector3, Vector3, Vector3, Vector3)> getQVertices = (x, y) => {
            return dir switch {
                0 => (new Vector3(0.0f, 0.0f, 0.0f), 
                    new Vector3(0.0f, 0.0f, x), 
                    new Vector3(0.0f, y, x), 
                    new Vector3(0.0f, y, 0.0f)),
                1 => (new Vector3(0.0f, 0.0f, 0.0f),
                    new Vector3(0.0f, 0.0f, x),
                    new Vector3(y, 0.0f, x),
                    new Vector3(y, 0.0f, 0.0f)),
                2 => (new Vector3(0.0f, 0.0f, 0.0f),
                    new Vector3(0.0f, y, 0.0f),
                    new Vector3(x, y, 0.0f),
                    new Vector3(x, 0.0f, 0.0f)),
                _ => (Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO),
            };
        };

        Quad runQuad = null;
        for (int i = 0; i < gridSize; i++) {
            runQuad = null;
            for (int j = 0; j < gridSize; j++) {
                if (example) {
                    await Animate.Move(cursorQuad.Transform, qorigin + new Vector3(0.0f, i, j), 0.5f);
                }
                if (slice[i, j] == 0) {
                    if (example) {
                        cursorQuad.Outline = cursorInactiveCol;
                    }
                } else {
                    int run = 0;
                    int runNr = slice[i, j];
                    if (example) {
                        cursorQuad.Outline = cursorActiveCol;
                    }
                    var gQuad = new Quad();
                    gQuad.Vertices = dirQuad[dir];
                    Vector3 llofst = dir == 0 ? new Vector3(0.0f, i, j) : dir == 1 ? new Vector3(i, 0.0f, j) : new Vector3(j, i, 0.0f);
                    gQuad.Transform.Pos = qorigin2 + llofst - new Vector3(0.02f, 0.02f, 0.02f);
                    gQuad.Color = genQuadCol.WithA(silent ? 0.0f : 0.5f);
                    world.CreateInstantly(gQuad);
                    runQuad = gQuad;
                    greedyQuads.Add((gQuad, dir, sliceIdx));

                    quadCount += 1;
                    onQuad?.Invoke();

                    while (j+run < gridSize && (withFix && slice[i, j+run] == runNr || !withFix && slice[i, j+run] != 0)) {
                        await Animate.InterpF(x => {
                            runQuad.Vertices = getQVertices(run+x, 1.0f);
                        }, 0.0f, 1.0f, example ? 0.5f : silent ? 0.0f : 0.1f);
                        run++;
                    }
                    int height = 1;
                    while (true) {
                        if (i+height >= gridSize) break;
                        for (int k = j; k < j + run; k++) {
                            //await Time.WaitSeconds(example ? 0.1f : 0.0f);
                            if (withFix) {
                                if (slice[i+height, k] != runNr) {
                                    goto heightDone;
                                }
                            } else {
                                if (slice[i+height, k] == 0) {
                                    goto heightDone;
                                }
                            }
                        }
                        await Animate.InterpF(x => {
                            runQuad.Vertices = getQVertices(run, height + x);
                        }, 0.0f, 1.0f, example ? 0.5f : silent ? 0.0f : 0.1f);
                        height++;
                    }
    heightDone:
                    for (int k = 0; k < run; k++) 
                    for (int l = 0; l < height; l++) 
                    {
                        slice[i+l, j+k] = 0;
                        if (example) {
                            unsetSliceBit(j+k, i+l);
                        }
                    }
                    j += run-1;
                }
                await Time.WaitSeconds(example ? 0.5f : 0.0f);
            }
        }
    }
    quadCount = 0;
    await doSlice(1, 0, example: true);
    await Time.WaitSeconds(1.0f);
    _ = Animate.InterpF(x => {
        cursorQuad.Outline = cursorActiveCol.WithA(1.0f - x);
        remainingQuads.ForEach(y => {
            y.q.Color = y.q.Color.WithA(1.0f - x);
            y.q.Outline = y.q.Outline.WithA(1.0f - x);
        });
    }, 0.0f, 1.0f, 0.5f);
    await Time.WaitSeconds(1.0f);

    foreach (var st in sliceText) {
        world.Destroy(st);
    }
    foreach (var gt in gridText) {
        world.Destroy(gt);
    }
    gridLines.Where(x => x.dir == 1 || (x.dir == 0 && x.coord.x != 1) || (x.dir == 2 && x.coord.y != 1)).ToList().ForEach(x => x.l.Color = x.l.Color.WithA(gridAlpha));

    await Animate.InterpF(x => {
        cam.Transform.Pos = Vector3.Lerp(sPos, targetPos, x);
        cam.Transform.Rot = Quaternion.Slerp(startRot, targetRot, x);
    }, 1.0f, 0.0f, 2.0f);

    // greedy begin
    _ = Animate.Offset(quadCountText.Transform, (0.0f, 60.0f), 1.0f);
    await Animate.Offset(quadCountText2.Transform, (0.0f, 60.0f), 1.0f);
    var quadCountText3 = new Text2D($"Greedy quads: {quadCount}");
    quadCountText3.Color = Color.WHITE;
    quadCountText3.Size = 30;
    quadCountText3.Anchor = (-0.45f, 0.35f);
    quadCountText3.Transform.Pos = (0.0f, 0.0f);
    await world.CreateFadeIn(quadCountText3, 2.0f);

    _ = Animate.OrbitAndLookAt(cam.Transform, Vector3.UP, Vector3.ZERO, -180.0f, 7.5f);

    for (int ddir = 0; ddir < 3; ddir++) {
        for (int sslice = 0; sslice < gridSize+1; sslice++) {
            if (ddir == 0 && sslice <= 1) continue;
            await doSlice(sslice, ddir, example: false, onQuad: () => {
                quadCountText3.Text = "Greedy quads: " + (quadCount).ToString();
            });
        }
    }

    foreach (var gquad in greedyQuads) {
        _ = Animate.InterpF(x => {
            gquad.q.Color = gquad.q.Color.WithA(x);
        }, gquad.q.Color.A, 1.0f, 1.0f);
    }

    await Animate.OrbitAndLookAt(cam.Transform, Vector3.UP, Vector3.ZERO, -180.0f, 7.5f);

    await Time.WaitSeconds(1.0f);

    // FIX BROKEN QUAD

    var lineS1 = minCorner + new Vector3(2.0f - 0.03f, 6.0f, 0.0f);
    var lineE1 = minCorner + new Vector3(2.0f - 0.03f, 6.0f, 8.0f);
    var lineS2 = minCorner + new Vector3(2.0f - 0.03f, 7.0f-0.03f, 0.0f);
    var lineE2 = minCorner + new Vector3(2.0f - 0.03f, 7.0f-0.03f, 8.0f);

    var lines = new (Vector3, Vector3)[] {
        (lineS1, lineE1),
        (lineS2, lineE2),
    };

    var lines3d = new List<Line3D>();
    foreach (var rline in lines) {
        var repLine = new Line3D();
        repLine.Vertices = new Vector3[] {
            rline.Item1,
            rline.Item2,
        };
        repLine.Width = 3.0f;
        repLine.Color = new Color(0.5f, 0.1f, 2.0f, 0.0f);
        world.CreateInstantly(repLine);
        lines3d.Add(repLine);
    }

    await Time.WaitSeconds(1.0f);

    gridLines.Where(x => x.dir == 1 || (x.dir == 0 && x.coord.x != 2) || (x.dir == 2 && x.coord.y != 2)).ToList().ForEach(x => x.l.Color = x.l.Color.WithA(0.0f));

    List<Task> ltasks = new();
    foreach (var rline in lines3d) {
        ltasks.Add(Animate.InterpF(x => {
            rline.Color = rline.Color.WithA(x);
        }, rline.Color.A, 1.0f, 1.0f));
    }
    await Task.WhenAll(ltasks);

    await Time.WaitSeconds(1.0f);

    await Animate.InterpF(x => {
        cam.Transform.Pos = Vector3.Lerp(sPos, targetPos + new Vector3(1.0f, 0.0f, 0.0f), x);
        cam.Transform.Rot = Quaternion.Slerp(startRot, targetRot, x);
    }, 0.0f, 1.0f, 2.0f);

    await Animate.InterpF(x => {
        greedyQuads.Where(gq => gq.dir != 0 || gq.slice != 2).ToList().ForEach(gq => {
            gq.q.Color = gq.q.Color.WithA(x);
            gq.q.Outline = gq.q.Outline.WithA(x);
        });
    }, 1.0f, 0.0f, 0.5f);

    await Time.WaitSeconds(1.0f);

    updateSlice(0, 2);

    for (int i = 0; i < gridSize; i++)
    for (int j = 0; j < gridSize; j++) {
        sliceText[i, j] = world.Clone(slice[j, i] > 0 ? s1 : s0);
        sliceText[i, j].Transform.Pos = gridPositions[j, i];
        sliceText[i, j].Color = slice[j, i] > 0 ? redCol : Color.WHITE;
        world.CreateInstantly(sliceText[i, j]);
    }

    ltasks.Clear();
    for (int i = 0; i < 8; i++)
    for (int j = 0; j < 8; j++) {
        ltasks.Add(Animate.Move(sliceText[i, j].Transform, ofs + new Vector2(-i*40.0f, j*40.0f), 0.5f));
    }
    await Task.WhenAll(ltasks);

    await Time.WaitSeconds(1.0f);

    s2.Color = genBlueCol;
    for (int i = 0; i < gridSize; i++)
    for (int j = 0; j < gridSize; j++) {
        if (slice[j, i] == 2) {
            var lltask = Animate.CreateMorph(sliceText[i, j], s2, 0.5f);
            var localI = i;
            var localJ = j;
            _ = lltask.ContinueWith(x => {
                sliceText[localI, localJ] = lltask.Result;
            });
        }
    }

    await Time.WaitSeconds(1.0f);

    // make remaining quads disappear
    cursorQuad.Transform.Pos = qorigin + new Vector3(2.0f, 0.0f, -gridSize);
    cursorQuad.Outline = cursorInactiveCol;
    await Animate.InterpF(x => {
        greedyQuads.Where(gq => gq.dir == 0 && gq.slice == 2).ToList().ForEach(gq => {
            gq.q.Color = gq.q.Color.WithA(x);
            gq.q.Outline = gq.q.Outline.WithA(x);
        });
        lines3d.ForEach(l => l.Color = l.Color.WithA(x));

        cursorQuad.Outline = cursorActiveCol.WithA(1.0f - x);
    }, 1.0f, 0.0f, 0.5f);
    greedyQuads.ForEach(gq => {
        world.Destroy(gq.q);
    });
    greedyQuads.Clear();

    quadCount = greedyQuads.Count;
    quadCountText3.Text = "Greedy quads: " + (quadCount).ToString();

    await doSlice(2, 0, example: true, withFix: true, onQuad: () => {
        quadCountText3.Text = "Greedy quads: " + (quadCount).ToString();
    });

    await Time.WaitSeconds(1.0f);

    _ = Animate.InterpF(x => {
        cursorQuad.Outline = cursorActiveCol.WithA(1.0f - x);
        foreach (var st in sliceText) {
            st.Color = st.Color.WithA(1.0f - x);
        }
    }, 0.0f, 1.0f, 0.5f);

    await Animate.InterpF(x => {
        cam.Transform.Pos = Vector3.Lerp(sPos, targetPos + new Vector3(1.0f, 0.0f, 0.0f), x);
        cam.Transform.Rot = Quaternion.Slerp(startRot, targetRot, x);
    }, 1.0f, 0.0f, 2.0f);

    for (int ddir = 0; ddir < 3; ddir++) {
        for (int sslice = 0; sslice < gridSize+1; sslice++) {
            if (ddir == 0 && sslice == 2) continue;
            await doSlice(sslice, ddir, example: false, onQuad: () => {
                quadCountText3.Text = "Greedy quads: " + (quadCount).ToString();
            }, silent: true, withFix: true);
        }
    }

    await Animate.InterpF(x => {
        foreach (var quad in greedyQuads) {
            quad.q.Color = quad.q.Color.WithA(MathF.Max(x, quad.q.Color.A));
            quad.q.Outline = quad.q.Outline.WithA(MathF.Max(x, quad.q.Outline.A));
        }
    }, 0.0f, 1.0f, 0.5f);

    // end

    //await Animate.InterpF(x => line.Width = x, 1.0f, 5.0f, 1.0f);
    //await Animate.Offset(cube.Transform, new Vector3(10.0f, 0.0f, 0.0f), 1.0f);

    await Time.WaitSeconds(15.0f);
}
}
