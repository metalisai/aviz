using System;
using System.Linq;
using AnimLib;
using System.Collections.Generic;
using System.Threading.Tasks;
using CommunityToolkit.HighPerformance;
using MathNet.Numerics.LinearAlgebra;
using System.Threading;
using ImgSharp = SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;
using SixLabors.ImageSharp.Processing;

using MyCMS = FasterMarchingSquares;

public class CubicalMS : AnimationBehaviour
{
    World world;
    Animator animator;

    Color[] cornerColors = new Color[] {
        new Color(0.0f, 0.0f, 0.0f, 1.0f),
        new Color(1.0f, 0.0f, 0.0f, 1.0f),
        new Color(1.0f, 1.0f, 0.0f, 1.0f),
        new Color(0.0f, 1.0f, 0.0f, 1.0f),
        new Color(0.0f, 0.0f, 1.0f, 1.0f),
        new Color(1.0f, 0.0f, 1.0f, 1.0f),
        new Color(0.5f, 0.5f, 0.5f, 1.0f),
        new Color(0.0f, 1.0f, 1.0f, 1.0f),
    };


public void Init(AnimationSettings settings) {
    settings.Name = "My animation";
    // animation length must be bounded
    // (it gets "baked" to allow seeking whole animation in editor)
    //settings.MaxLength = 1600.0f; 
    settings.MaxLength = 1200.0f;
    //settings.Width = 2560;
    //settings.Height = 1440;
    settings.Width = 960;
    settings.Height = 540;
}

/*
// edges relative to lower left corner
// vertex will be placed between these two points
(Vector2 s, Vector2 e)[] quadEdges = new (Vector2, Vector2)[] {
    (new Vector2(0, 0), new Vector2(1, 0)),
    (new Vector2(1, 0), new Vector2(1, 1)),
    (new Vector2(1, 1), new Vector2(0, 1)),
    (new Vector2(0, 1), new Vector2(0, 0)),
};*/

static float opUnion(float d1, float d2) {
    return MathF.Min(d1, d2);
}

static float opSmoothUnion(float d1, float d2, float k = 0.1f) {
    float h = float.Clamp(0.5f + 0.5f*(d2-d1)/k, 0.0f, 1.0f);
    return float.Lerp(d2, d1, h) - k*h*(1.0f-h);
}

static float opSubtract(float d1, float d2) {
    return MathF.Max(-d1, d2);
}

static float opXor(float d1, float d2) {
    return MathF.Max(MathF.Min(d1, d2), -MathF.Max(d1, d2));
}

static float sdSphere(Vector3 p, float r)
{
    return p.Length - r;
}

static float sdPyramid(Vector3 position, float halfWidth, float halfDepth, float halfHeight) {
    position.y += halfHeight;
    position.x = float.Abs(position.x);
    position.z = float.Abs(position.z);
    var d1 = new Vector3(float.Max(position.x - halfWidth, 0.0f), position.y, float.Max(position.z - halfDepth, 0.0f));
    var n1 = new Vector3(0.0f, halfDepth, 2.0f * halfHeight);
    float k1 = Vector3.Dot(n1, n1);
    float h1 = Vector3.Dot(position - new Vector3(halfWidth, 0.0f, halfDepth), n1) / k1;
    var n2 = new Vector3(k1, 2.0f * halfHeight * halfWidth, -halfDepth * halfWidth);
    float m1 = Vector3.Dot(position - new Vector3(halfWidth, 0.0f, halfDepth), n2) / Vector3.Dot(n2, n2);
    var clampV1 = position - n1 * h1 - n2 * float.Max(m1, 0.0f);
    var clampTo1 = new Vector3(halfWidth, 2.0f * halfHeight, halfDepth);
    var clamped1 = new Vector3(
        float.Clamp(clampV1.x, 0.0f, clampTo1.x),
        float.Clamp(clampV1.y, 0.0f, clampTo1.y),
        float.Clamp(clampV1.z, 0.0f, clampTo1.z)
    );
    var d2 = position - clamped1;
    var n3 = new Vector3(2.0f * halfHeight, halfWidth, 0.0f);
    float k2 = Vector3.Dot(n3, n3);
    float h2 = Vector3.Dot(position - new Vector3(halfWidth, 0.0f, halfDepth), n3) / k2;
    var n4 = new Vector3(-halfWidth * halfDepth, 2.0f * halfHeight * halfDepth, k2);
    float m2 = Vector3.Dot(position - new Vector3(halfWidth, 0.0f, halfDepth), n4) / Vector3.Dot(n4, n4);    
    var clampV = position - n3 * h2 - n4 * float.Max(m2, 0.0f);
    var clampTo = new Vector3(halfWidth, 2.0f * halfHeight, halfDepth);
    var clamped = new Vector3(
        float.Clamp(clampV.x, 0.0f, clampTo.x),
        float.Clamp(clampV.y, 0.0f, clampTo.y),
        float.Clamp(clampV.z, 0.0f, clampTo.z)
    );
    var d3 = position - clamped;
    float d = float.Sqrt(float.Min(float.Min(Vector3.Dot(d1, d1), Vector3.Dot(d2, d2)), Vector3.Dot(d3, d3)));
    return float.Max(float.Max(h1, h2), -position.y) < 0.0 ? -d : d;
}

static float sdBox(Vector3 p, Vector3 b )
{
    var q = (p).Abs - b;
    var t1 = Vector3.Max(q, new Vector3(0.0f));
    return t1.Length + MathF.Min(MathF.Max(q.x,MathF.Max(q.y,q.z)), 0.0f);
}

static float sdTorus(Vector3 p/*, vec2 t */)
{
    Vector2 t = new Vector2(5.0f, 2.0f);
    float pxzm = new Vector2(p.x, p.z).Length;
    Vector2 q = new Vector2(pxzm-t.x, p.y);
    return q.Length - t.y;
}

static Vector3 sdTorusNormal(Vector3 p)
{
    float R = 5.0f;
    Vector2 c = Vector2.ZERO;

    Vector2 v = new Vector2(p.x, p.z);
    float magV = v.Length;
    Vector2 a = c + v / magV * R;

    //return (p - a).normalized;
    return (p - new Vector3(a.x, 0.0f, a.y)).Normalized;
}

public static float Evaluate2(Vector3 pos) {
    //return sdTorus(pos);
    var torusBox = opUnion(sdBox(pos, new Vector3(4.11111111111f, 6.111111111f, 4.1111111111111f)), sdTorus(pos));
    var subSphere = opUnion(torusBox, sdSphere(pos - new Vector3(0.0f, 5.0f, -2.0f), 3.0f));
    subSphere = opSubtract(sdSphere(pos - new Vector3(0.0f, -5.0f, -2.0f), 3.0f), subSphere);
    return subSphere;
    //return torusBox;
}

public static float Evaluate(Vector3 pos) {
    //return sdTorus(pos);
    var torusBox = opUnion(sdBox(pos, new Vector3(4.11111111111f, 6.111111111f, 4.1111111111111f)), sdTorus(pos));
    var subSphere = opUnion(torusBox, sdSphere(pos - new Vector3(0.0f, 5.0f, -2.0f), 3.0f));
    subSphere = opSubtract(sdSphere(pos - new Vector3(0.0f, -5.0f, -2.0f), 3.0f), subSphere);
    return subSphere;
    //return torusBox;
}

public static Vector3 CalcNormal(Func<Vector3, float> evalf, Vector3 pos, float eps = 0.001f) {
    float dx = evalf(pos + new Vector3(eps, 0, 0)) - evalf(pos - new Vector3(eps, 0, 0));
    float dy = evalf(pos + new Vector3(0, eps, 0)) - evalf(pos - new Vector3(0, eps, 0));
    float dz = evalf(pos + new Vector3(0, 0, eps)) - evalf(pos - new Vector3(0, 0, eps));
    return new Vector3(dx, dy, dz).Normalized;
}

/*public static Vector3 CalcNormal(Func<Vector3, float> evalf, Vector3 pos, float eps = 0.001f) {
    //return sdTorusNormal(pos);
    // use gradient approximation
    float d = evalf(pos);
    float dx = evalf(pos + new Vector3(eps, 0.0f, 0.0f));
    float dy = evalf(pos + new Vector3(0.0f, eps, 0.0f));
    float dz = evalf(pos + new Vector3(0.0f, 0.0f, eps));
    return new Vector3(dx - d, dy - d, dz - d).Normalized;
}*/

public static Vector3 EvaluateNormal(Vector3 pos) {
    return CalcNormal(Evaluate, pos);
}

public static (Vector3 s, Vector3 e)[] CellOuterEdges(Vector3 min, float size) {
    var edges = new (Vector3, Vector3)[12];
    for (int i = 0; i < 12; i++) {
        int si = CMS.cubeEdgeCorners[i].s;
        int ei = CMS.cubeEdgeCorners[i].e;
        var s = min + CMS.cornerOffsets[si] * size;
        var e = min + CMS.cornerOffsets[ei] * size;
        edges[i] = (s, e);
    }
    return edges;
}

public static (Vector3 s, Vector3 e)[] CellInnerEdges(Vector3 min, float size) {
    (Vector3 s, Vector3 e)[] edges = [
        (min + new Vector3(0.5f, 0.5f, 0.0f) * size, min + new Vector3(0.5f, 0.5f, 1.0f) * size),
        (min + new Vector3(0.5f, 0.0f, 0.5f) * size, min + new Vector3(0.5f, 1.0f, 0.5f) * size),
        (min + new Vector3(0.0f, 0.5f, 0.5f) * size, min + new Vector3(1.0f, 0.5f, 0.5f) * size),

        (min + new Vector3(0.0f, 0.5f, 0.0f) * size, min + new Vector3(1.0f, 0.5f, 0.0f) * size),
        (min + new Vector3(0.5f, 0.0f, 0.0f) * size, min + new Vector3(0.5f, 1.0f, 0.0f) * size),
        (min + new Vector3(0.0f, 0.5f, 0.0f) * size, min + new Vector3(0.0f, 0.5f, 1.0f) * size),
        (min + new Vector3(0.0f, 0.0f, 0.5f) * size, min + new Vector3(0.0f, 1.0f, 0.5f) * size),
        (min + new Vector3(0.0f, 0.0f, 0.5f) * size, min + new Vector3(1.0f, 0.0f, 0.5f) * size),
        (min + new Vector3(0.5f, 0.0f, 0.0f) * size, min + new Vector3(0.5f, 0.0f, 1.0f) * size),
        (min + new Vector3(0.0f, 0.5f, 1.0f) * size, min + new Vector3(1.0f, 0.5f, 1.0f) * size),
        (min + new Vector3(0.5f, 0.0f, 1.0f) * size, min + new Vector3(0.5f, 1.0f, 1.0f) * size),
        (min + new Vector3(1.0f, 0.5f, 0.0f) * size, min + new Vector3(1.0f, 0.5f, 1.0f) * size),
        (min + new Vector3(1.0f, 0.0f, 0.5f) * size, min + new Vector3(1.0f, 1.0f, 0.5f) * size),
        (min + new Vector3(0.0f, 1.0f, 0.5f) * size, min + new Vector3(1.0f, 1.0f, 0.5f) * size),
        (min + new Vector3(0.5f, 1.0f, 0.0f) * size, min + new Vector3(0.5f, 1.0f, 1.0f) * size),
    ];
    return edges;
}

public HermiteIntersection[] GetIntersections((Vector3 start, Vector3 end) edge, Func<Vector3, float> evalF = null, Func<Vector3, Vector3> normalF = null, int steps = 32) {
    Vector3 step = (edge.end - edge.start) * (1.0f/steps);
    float lastSample = evalF != null ? evalF(edge.start) : Evaluate(edge.start);
    List<HermiteIntersection> intersections = new List<HermiteIntersection>();
    for (int i = 0; i < steps+1; i++) {
        Vector3 samplePos = edge.start + step * i;
        float sample = evalF != null ? evalF(samplePos) : Evaluate(samplePos);

        if ((MathF.Sign(sample) < 0) != (MathF.Sign(lastSample) < 0)) {
            HermiteIntersection intersection = new HermiteIntersection();
            intersection.pos = samplePos;
            intersection.normal = normalF != null ? normalF(samplePos) : EvaluateNormal(samplePos);
            if (Vector3.Dot(intersection.normal, intersection.normal) != 1.0f) {
                var ap = samplePos + new Vector3(0.0315f, 0.0315f, 0.0315f);
                intersection.normal = normalF != null ? normalF(ap) :  EvaluateNormal(ap);
            }
            intersection.t = (float)i / steps;
            intersections.Add(intersection);
        }

        lastSample = sample;
    }
    return intersections.ToArray();
}

static public void MarchSquare(float[] cornerSamples, int[] buffer, out int outputCount) {
    bool sample0 = cornerSamples[0] < 0.0f;
    bool sample1 = cornerSamples[1] < 0.0f;
    bool sample2 = cornerSamples[2] < 0.0f;
    bool sample3 = cornerSamples[3] < 0.0f;
    MarchSquare(new bool[] { sample0, sample1, sample2, sample3 }, buffer, out outputCount);
}

static public void MarchSquare(bool[] cornerSamples, int[] buffer, out int outputCount) {
    bool sample0 = cornerSamples[0];
    bool sample1 = cornerSamples[1];
    bool sample2 = cornerSamples[2];
    bool sample3 = cornerSamples[3];
    int caseId = (sample0 ? 1 : 0) | (sample1 ? 2 : 0) | (sample2 ? 4 : 0) | (sample3 ? 8 : 0);

    outputCount = 0;
    for (int i = 0; i < 4; i+=2) {
        int startEdge = CMS.quadSegments[caseId, i];
        if (startEdge == -1) {
            return;
        }
        int endEdge = CMS.quadSegments[caseId, i+1];
        buffer[outputCount++] = startEdge;
        buffer[outputCount++] = endEdge;
    }
}

static bool lineListHasIntersection(Vector2[] vertices, int vcount, int[] indices, int icount) {
    int lineCount = indices.Length / 2;
    for (int i = 0; i < lineCount; i+=2) {
        var v1 = vertices[indices[i]];
        var v2 = vertices[indices[i+1]];

        for (int j = i+2; j < lineCount; j+=2) {
            var v3 = vertices[indices[j]];
            var v4 = vertices[indices[j+1]];

            if (AMath.SegmentsIntersect(v1, v2, v3, v4)) {
                return true;
            }
        }
    }
    return false;
}

const float SHARP_FEATURE_ANGLE_THRESHOLD = 0.99f;

public struct MSVertex2D {
    public bool isNew; // is this a new vertex generated by sharp feature detection
    public Vector2 newVertex; // coordinate in face-space, valid if isNew is true
    public int inputIndex; // input intersection point index, valid if isNew is false
}

// cell face vertex (discriminated union of edge intersection point and new vertex)
public struct CMSVertex {
    public bool isNew;
    public Vector3 newVertex; // a new vertex generated by sharp feature detection
    public int edgeIndex; // cell edge index

    public Vector3 normal;
}

public List<(Vector3 pos, Vector3 n)> GetGridIntersections(Vector3 min, float cellSize, int size) {
List<(Vector3 pos, Vector3 n)> intersections = new List<(Vector3, Vector3)>();
    for (int dir = 0; dir < 3; dir++) {
        var dirVec = dir switch {
            0 => Vector3.RIGHT,
            1 => Vector3.UP,
            2 => Vector3.FORWARD,
            _ => throw new Exception("Invalid direction")
        };
        for (int i = 0; i < size; i++)
        for (int j = 0; j < size; j++) {
            var offsetVec = dir switch {
                0 => Vector3.UP * i + Vector3.FORWARD * j,
                1 => Vector3.RIGHT * i + Vector3.FORWARD * j,
                2 => Vector3.RIGHT * i + Vector3.UP * j,
                _ => throw new Exception("Invalid direction")
            };
            var start = min + offsetVec * cellSize;
            var end = start + dirVec * cellSize * size;
            var edge = (start, end);
            var edgeIntersections = GetIntersections(edge);
            intersections.AddRange(edgeIntersections.Select(i => (i.pos, i.normal)));
        }
    }
    return intersections;
}

static public void MarchSquareComplex(bool[] cornerSamples, Vector2[] intersectionNormals, Vector2[] intersectionPositions, MSVertex2D[] outputVertices, int[] outputSegments, out int outVCount, out int outICount) {
    outVCount = 0;
    outICount = 0;

    bool sample0 = cornerSamples[0];
    bool sample1 = cornerSamples[1];
    bool sample2 = cornerSamples[2];
    bool sample3 = cornerSamples[3];
    int caseId = (sample0 ? 1 : 0) | (sample1 ? 2 : 0) | (sample2 ? 4 : 0) | (sample3 ? 8 : 0);

    if (caseId == 0 || caseId == 15) {
        return;
    }

    Vector2? detectSharpFeature(int startEdge, int endEdge) {
        var p1 = intersectionPositions[startEdge];
        var p2 = intersectionPositions[endEdge];

        Debug.Assert(p1.x >= 0.0f && p1.x <= 1.0f);
        Debug.Assert(p1.y >= 0.0f && p1.y <= 1.0f);
        Debug.Assert(p2.x >= 0.0f && p2.x <= 1.0f);
        Debug.Assert(p2.y >= 0.0f && p2.y <= 1.0f);

        var normal1 = intersectionNormals[startEdge].Normalized;
        var normal2 = intersectionNormals[endEdge].Normalized;
        if (Vector2.Dot(normal1, normal2) > SHARP_FEATURE_ANGLE_THRESHOLD) {
            return null;
        }
        var tangent1 = normal1.PerpCw;
        var tangent2 = normal2.PerpCcw;
        // solution by inverse matrix (A^-1 * b)
        var det = tangent1.x*tangent2.y - tangent1.y*tangent2.x;
        if (det == 0.0f) {
            return null;
        }
        var t = (tangent2.y*(p2.x-p1.x) - tangent2.x*(p2.y-p1.y)) / det;
        var sharpFeature = p1 + tangent1 * t;
        sharpFeature.x = Math.Clamp(sharpFeature.x, 0.0f, 1.0f);
        sharpFeature.y = Math.Clamp(sharpFeature.y, 0.0f, 1.0f);
        return sharpFeature;
    }

    var qSegs = new Span2D<int>(CMS.quadSegments).GetRow(caseId).ToArray();
    if (caseId != 5 && caseId != 10) {
        int startEdge = CMS.quadSegments[caseId, 0];
        int endEdge = CMS.quadSegments[caseId, 1];
        var sharpFeature = detectSharpFeature(startEdge, endEdge);
        outVCount = sharpFeature == null ? 2 : 3;
        outICount = sharpFeature == null ? 2 : 4;
        outputVertices[0] = new MSVertex2D() { 
            isNew = false, 
            inputIndex = startEdge 
        };
        outputVertices[1] = new MSVertex2D() { 
            isNew = false, 
            inputIndex = endEdge 
        };
        if (sharpFeature != null) {
            outputVertices[2] = new MSVertex2D() { 
                isNew = true, 
                newVertex = sharpFeature.Value 
            };
            outputSegments[0] = 0;
            outputSegments[1] = 2;
            outputSegments[2] = 2;
            outputSegments[3] = 1;
        } else {
            outputSegments[0] = 0;
            outputSegments[1] = 1;
        }
    } else { // ambiguous case
        var altSegs = new Span2D<int>(CMS.alternativeSegments).GetRow(caseId == 5 ? 0 : 1).ToArray();

        // there could be a gap between corners
        // or they could be connected (for example see case 5 in marching squares)
        // detecting sharp feature might make one of the cases impossible
        // so we consider both cases and pick the one that has no intersections
        int[][] possibleSegments = new int[2][];
        possibleSegments[0] = qSegs;
        possibleSegments[1] = altSegs;

        MSVertex2D[,] potentialVertices = new MSVertex2D[2, 6];
        int[,] potentialSegments = new int[2, 8];
        
        int[] vCounts = new int[2];
        int[] iCounts = new int[2];

        for (int si = 0; si < 2; si++) {
            int s1 = possibleSegments[si][0];
            int e1 = possibleSegments[si][1];
            var sFeature1 = detectSharpFeature(s1, e1);
            int s2 = possibleSegments[si][2];
            int e2 = possibleSegments[si][3];
            var sFeature2 = detectSharpFeature(s2, e2);

            potentialVertices[si, 0] = new MSVertex2D() { 
                isNew = false, 
                inputIndex = s1 
            };
            potentialVertices[si, 1] = new MSVertex2D() { 
                isNew = false, 
                inputIndex = e1 
            };
            potentialVertices[si, 2] = new MSVertex2D() { 
                isNew = false, 
                inputIndex = s2 
            };
            potentialVertices[si, 3] = new MSVertex2D() { 
                isNew = false, 
                inputIndex = e2 
            };
            int vi = 4;
            int ii = 0;
            if (sFeature1 != null) {
                potentialVertices[si, vi++] = new MSVertex2D() { 
                    isNew = true, 
                    newVertex = sFeature1.Value 
                };
                potentialSegments[si, ii++] = 0;
                potentialSegments[si, ii++] = vi-1;
                potentialSegments[si, ii++] = vi-1;
                potentialSegments[si, ii++] = 1;
            }
            if (sFeature2 != null) {
                potentialVertices[si, vi++] = new MSVertex2D() { 
                    isNew = true, 
                    newVertex = sFeature2.Value 
                };
                potentialSegments[si, ii++] = 2;
                potentialSegments[si, ii++] = vi-1;
                potentialSegments[si, ii++] = vi-1;
                potentialSegments[si, ii++] = 3;
            }
            vCounts[si] = vi;
            iCounts[si] = ii;
        }

        var vs1 = potentialVertices.GetRow(0).ToArray().Select(
            v => v.isNew ? v.newVertex : intersectionPositions[v.inputIndex]
        ).ToArray();
        var is1 = potentialSegments.GetRow(0).ToArray();
        var vs2 = potentialVertices.GetRow(1).ToArray().Select(
            v => v.isNew ? v.newVertex : intersectionPositions[v.inputIndex]
        ).ToArray();
        var is2 = potentialSegments.GetRow(1).ToArray();
        var hasIntersection1 = lineListHasIntersection(vs1, vCounts[0], is1, iCounts[0]);
        var hasIntersection2 = lineListHasIntersection(vs2, vCounts[1], is2, iCounts[1]);

        int selectedOption = 0;
        if (!hasIntersection1) {
            selectedOption = 0;
        } else if (!hasIntersection2) {
            selectedOption = 1;
        } else {
            //throw new Exception("Both cases have intersection");
            Debug.Error("Both cases have intersection");
        }

        outVCount = vCounts[selectedOption];
        outICount = iCounts[selectedOption];
        for (int i = 0; i < outVCount; i++) {
            outputVertices[i] = potentialVertices[selectedOption, i];
        }
        for (int i = 0; i < outICount; i++) {
            outputSegments[i] = potentialSegments[selectedOption, i];
        }
    }

    // TODO: detect pair count
    // TODO: detect sharp feature for each pair (if angle above threshold)
    // TODO: if two pairs, resolve ambiguity
}


public async Task<Line3D> CubicalMarchingSquares2(Vector3[,,] sLoc, float cellSize, Func<Vector3, float> sampleF, Func<Vector3, Vector3> normalF, float stepWait = 0.0f, List<Vector3>[,,] cellVertices = null) {
    int gridSize = sLoc.GetLength(0) - 1;

    List<Vector3> vertices = new List<Vector3>();

    var lines = new Line3D();
    lines.Color = Color.YELLOW;
    lines.Width = 6.0f;
    world.CreateInstantly(lines);

    for (int i = 0; i < gridSize; i++)
    for (int j = 0; j < gridSize; j++)
    for (int k = 0; k < gridSize; k++) {
        float[] cornerSamples = new float[8];
        Vector3[] cornerPositions = new Vector3[8];
        for (int c = 0; c < CMS.cornerOffsetsI.Length; c++) {
            var o = CMS.cornerOffsetsI[c];
            cornerPositions[c] = sLoc[i + o.x, j + o.y, k + o.z];
            cornerSamples[c] = sampleF(cornerPositions[c]);
        }

        for (int faceId = 0; faceId < 6; faceId++) {
            float[] faceSampleBuf = new float[4];
            faceSampleBuf[0] = cornerSamples[CMS.faceCubeCorners[faceId, 0]];
            faceSampleBuf[1] = cornerSamples[CMS.faceCubeCorners[faceId, 1]];
            faceSampleBuf[2] = cornerSamples[CMS.faceCubeCorners[faceId, 2]];
            faceSampleBuf[3] = cornerSamples[CMS.faceCubeCorners[faceId, 3]];

            Vector2[] ns = new Vector2[4];
            Vector2[] ps = new Vector2[4];
            Vector3[] ps3d = new Vector3[4];
            MSVertex2D[] outputVertices = new MSVertex2D[6];
            int[] segmentBuffer = new int[8];
            int vCount;
            int iCount;

            for (int ei = 0; ei < 4; ei++) {
                var (c0, c1) = CMS.quadEdgeCorners[ei];
                if (MathF.Sign(faceSampleBuf[c0]) < 0.0f != MathF.Sign(faceSampleBuf[c1]) < 0.0f) 
                {
                
                    var cubeEdge = CMS.faceEdgeToCubeEdge[faceId, ei];
                    var (cc0, cc1) = CMS.cubeEdgeCorners[cubeEdge];
                    var startP = cornerPositions[cc0];
                    var endP = cornerPositions[cc1];
                    var midP = (startP + endP) * 0.5f;
                    ps3d[ei] = midP;
                    var normal3d = normalF(midP);

                    var normal2d = CMS.ProjectOnFace(faceId, normal3d);
                    ns[ei] = normal2d;

                    var startP2d = CMS.quadCorners[CMS.quadEdgeCorners[ei].Item1];
                    var endP2d = CMS.quadCorners[CMS.quadEdgeCorners[ei].Item2];
                    var midP2d = (startP2d + endP2d) * 0.5f;
                    ps[ei] = midP2d;
                }
            }

            MarchSquareComplex(faceSampleBuf.Select(x => x < 0.0f).ToArray(), ns, ps, outputVertices, segmentBuffer, out vCount, out iCount);
            for (int ii = 0; ii < iCount; ii++) {
                var vi = segmentBuffer[ii];
                var v = outputVertices[vi];
                if (v.isNew) {
                    var new2d = v.newVertex;
                    var new3d = cornerPositions[0] + CMS.FacePosToCubePos(faceId, new2d)*cellSize;
                    vertices.Add(new3d);
                    if (cellVertices != null) {
                        cellVertices[i, j, k].Add(new3d);
                    }
                } else {
                    vertices.Add(ps3d[v.inputIndex]);
                    if (cellVertices != null) {
                        cellVertices[i, j, k].Add(ps3d[v.inputIndex]);
                    }
                }
            }
        }
        lines.Vertices = vertices.ToArray();
        await Time.WaitSeconds(stepWait);
    }
    return lines;
}

public Line3D CubicalMarchingSquares(float[,,] samples, float cellSize, Vector3 offset) {
    int gridSize = samples.GetLength(0) - 1;

    int segmentCount = 0;
    // edge indices within face quad
    int[] segmentBuffer = new int[4];

    float[] faceSampleBuf = new float[4];

    int[] vertexEdgeBuffer = new int[4*6];

    // DEBUG
    int[] vertexFaceBuffer = new int[4*6];

    List<Vector3> vertices = new List<Vector3>();

    for (int i = 0; i < gridSize; i++)
    for (int j = 0; j < gridSize; j++) 
    for (int k = 0; k < gridSize; k++) {
        float[] cornerSamples = new float[8];
        for (int c = 0; c < CMS.cornerOffsetsI.Length; c++) {
            var o = CMS.cornerOffsetsI[c];
            cornerSamples[c] = samples[i + o.x, j + o.y, k + o.z];
        }

        int vIdx = 0;
        for (int faceId = 0; faceId < CMS.faceCubeCorners.GetLength(0); faceId++) {
            faceSampleBuf[0] = cornerSamples[CMS.faceCubeCorners[faceId, 0]];
            faceSampleBuf[1] = cornerSamples[CMS.faceCubeCorners[faceId, 1]];
            faceSampleBuf[2] = cornerSamples[CMS.faceCubeCorners[faceId, 2]];
            faceSampleBuf[3] = cornerSamples[CMS.faceCubeCorners[faceId, 3]];

            MarchSquare(faceSampleBuf, segmentBuffer, out segmentCount);
            for (int eIdx = 0; eIdx < segmentCount; eIdx++) {
                vertexFaceBuffer[vIdx] = faceId; // DEBUG
                vertexEdgeBuffer[vIdx++] = CMS.faceEdgeToCubeEdge[faceId, segmentBuffer[eIdx]];
            }
        }

        Vector3[] vertexPositions = new Vector3[vIdx];
        for (int vi = 0; vi < vIdx; vi++) {
            var edge = vertexEdgeBuffer[vi];
            var face = vertexFaceBuffer[vi]; // DEBUG
            var (c0, c1) = CMS.cubeEdgeCorners[edge];
            var p0 = CMS.cornerOffsets[c0]*cellSize;
            var p1 = CMS.cornerOffsets[c1]*cellSize;
            var p = (p0 + p1) * 0.5f;

            var worldP = p + new Vector3(i, j, k)*cellSize + offset;
            var sphere = new Sphere(0.05f);
            sphere.Color = Color.ORANGE;
            sphere.Position = worldP;
            //world.CreateInstantly(sphere);
            vertexPositions[vi] = worldP;

            vertices.Add(worldP);

            if (vi % 2 == 1) {

                var line = new Line3D();
                line.Color = cornerColors[face]; // DEBUG
                line.Vertices = [vertexPositions[vi-1], vertexPositions[vi]];
                line.Width = 6.0f;
                //world.CreateInstantly(line);
            }
        }
    }

    var lines = new Line3D();
    lines.Color = Color.ORANGE;
    lines.Vertices = vertices.ToArray();
    lines.Width = 6.0f;
    return lines;

}

public Line3D CubicalMarchingSquares() {
    Vector3 offset = new Vector3(-8.0f, -8.0f, -8.0f);

    int gridSize = 32;
    float cellSize = 0.5f;

    float[,,] samples = new float[gridSize+1, gridSize+1, gridSize+1];
    //Vector3[,,] normals = new Vector3[gridSize+1, gridSize+1, gridSize+1];

    for (int i = 0; i < gridSize+1; i++)
    for (int j = 0; j < gridSize+1; j++) 
    for (int k = 0; k < gridSize+1; k++) {
        Vector3 p = new Vector3(i, j, k)*cellSize + offset;
        samples[i, j, k] = Evaluate(p);
        //normals[i, j, k] = sdTorusNormal(samplePos);

        /*if (j > 5 && j < 11) {
            var sphere = new Sphere();
            sphere.Radius = 0.05f;
            sphere.Color = samples[i, j, k] < 0.0f ? Color.ORANGE : Color.BLUE;
            sphere.Position = p;
            world.CreateInstantly(sphere);
        }*/
    }

    var lines = CubicalMarchingSquares(samples, cellSize, offset);
    return lines;
}


public Mesh DoCMS2(Func<Vector3, float> evalF, Func<Vector3, Vector3> normalF, int nmax = 0) {
    Debug.Warning("CMS2 is not implemented yet");
    List<(Vector3, Vector3)> intersectionsInternal = new();
    List<(Vector3, Vector3)> intersectionsFace = new();
    List<(Vector3, Vector3)> intersectionNormals = new();

    var tree = new CMSTree();
    var sw = new System.Diagnostics.Stopwatch();
    sw.Start();
    Func<(Vector3 s, Vector3 e), HermiteIntersection[]> getIntr = (pos) => {
        return GetIntersections(pos, evalF, normalF);
    };
    tree.Init(4, 8.0f, new Vector3(-4.0f), getIntr, nmax: 0);
    Debug.Log($"CMS2 tree init took {sw.ElapsedMilliseconds} ms");
    sw.Restart();
    tree.CalculateIntersections(tree.root, getIntr);
    Debug.Log($"CMS2 tree intersections took {sw.ElapsedMilliseconds} ms");

    List<CMSCell> cells = new List<CMSCell>();
    tree.getLeafCells(tree.root, cells);

    List<(Vector3 s, Vector3 e)> segments = new();

    Vector3[] meshVertices;
    int[] meshIndices;
    sw.Restart();
    Func<Vector3, bool> eval = (pos) => {
        return evalF(pos) < 0;
    };
    tree.ExtractSurface(out meshVertices, out meshIndices, eval);
    Debug.Log($"CMS2 tree surface extraction took {sw.ElapsedMilliseconds} ms");
    sw.Stop();

    var mesh = new Mesh();
    mesh.Vertices = meshVertices;
    mesh.Indices = meshIndices.Select(i => (uint)i).ToArray();
    mesh.Color = new Color(0.2f, 0.2f, 1.0f, 1.0f);

    return mesh;
}

CMSTree test;

public Mesh DoCMS() {
    List<(Vector3, Vector3)> intersectionsInternal = new();
    List<(Vector3, Vector3)> intersectionsFace = new();
    List<(Vector3, Vector3)> intersectionNormals = new();

    var tree = new CMSTree();
    test = tree;
    var sw = new System.Diagnostics.Stopwatch();
    sw.Start();
    Func<(Vector3 s, Vector3 e), HermiteIntersection[]> getIntr = (pos) => {
        return GetIntersections(pos, Evaluate, EvaluateNormal);
    };
    Debug.Log($"Start CMS");
    // init cells at fixed depth, subdivide until flat or nmax is reached
    tree.Init(8, 16.0f, new Vector3(-8.0f, -8.0f, -8.0f), getIntr, nmax: 1);
    Debug.Log($"CMS tree init took {sw.ElapsedMilliseconds} ms");
    sw.Restart();
    tree.CalculateIntersections(tree.root, getIntr);
    Debug.Log($"CMS tree intersections took {sw.ElapsedMilliseconds} ms");
    void edgeDebug() {
        void showEdgeTrees(CMSCell cell) {
            if (cell.children != null) {
                foreach (var c in cell.children) {
                    showEdgeTrees(c);
                }
            }
            void showEdgeTree(EdgeNode n) {
                if (n == null) return;
                if (n.first != null) {
                    showEdgeTree(n.first);
                    showEdgeTree(n.second);
                } else {
                    /*var line = new Line3D();
                    line.Color = Color.Random();
                    line.Vertices = [n.s, n.e];
                    line.Width = 2.0f;
                    world.CreateInstantly(line);*/
                    if (n.isInternal) {
                        intersectionsInternal.Add((n.s, n.e));
                    } else {
                        intersectionsFace.Add((n.s, n.e));
                    }
                    if (n.intersectionT.HasValue) {
                        intersectionNormals.Add((Vector3.Lerp(n.s, n.e, n.intersectionT.Value), n.intersectionNormal));
                    }
                }
            }
            foreach (var ft in cell.faceTrees) {
                if (ft == null) continue;
                foreach (var et in ft.edgeTrees) {
                    showEdgeTree(et);
                }
            }
        }
        foreach(var c in tree.root.children) {
            showEdgeTrees(c);
        }
    }
    edgeDebug();
    var intrLines = new Line3D();
    intrLines.Color = Color.RED;
    intrLines.Vertices = intersectionsInternal.SelectMany(i => new Vector3[] { i.Item1, i.Item2 }).ToArray();
    intrLines.Width = 3.0f;
    //world.CreateInstantly(intrLines);

    var faceLines = new Line3D();
    faceLines.Color = Color.VIOLET;
    faceLines.Vertices = intersectionsFace.SelectMany(i => new Vector3[] { i.Item1, i.Item2 }).ToArray();
    faceLines.Width = 3.0f;
    //world.CreateInstantly(faceLines);

    var normalLines = new Line3D();
    normalLines.Color = Color.GREEN;
    normalLines.Vertices = intersectionNormals.SelectMany(i => new Vector3[] { i.Item1, i.Item1 + 0.3f*i.Item2 }).ToArray();
    normalLines.Width = 5.0f;
    //world.CreateInstantly(normalLines);

    //await Time.WaitSeconds(1.0f);
    //return;

    List<CMSCell> cells = new List<CMSCell>();
    tree.getLeafCells(tree.root, cells);
    foreach (var cell in cells) {
        if (cell.size == 2.0f) continue;
        //var c = world.Clone(cube);
        //c.Position = cell.min;
        //c.Transform.Scale = new Vector3(cell.size, cell.size, cell.size);
        //world.CreateInstantly(c);
    }

    List<(Vector3 s, Vector3 e)> segments = new();

    Vector3[] meshVertices;
    int[] meshIndices;
    sw.Restart();

    Func<Vector3, bool> eval = (pos) => {
        return CubicalMS.Evaluate(pos) < 0;
    };

    tree.ExtractSurface(out meshVertices, out meshIndices, eval);
    Debug.Log($"CMS tree surface extraction took {sw.ElapsedMilliseconds} ms");
    sw.Stop();

    var mesh = new Mesh();
    mesh.Vertices = meshVertices;
    mesh.Indices = meshIndices.Select(i => (uint)i).ToArray();
    mesh.Color = new Color(0.2f, 0.2f, 1.0f, 1.0f);

    return mesh;
}

//List<(Vector3, Vector3)> normals = new();

public async Task FoldQuad(VisualEntity3D q, Vector3 pos, Vector3 pivot, Vector3 axis, float startAngle, float endAngle, double duration) {
    Vector3 d = pos - pivot;
    await Animate.InterpF(x => {
        var rot = Quaternion.AngleAxis(x, axis);
        q.Rotation = rot;
        var p = rot * d;
        q.Position = pivot + p;
    }, startAngle, endAngle, duration);
}

Task FoldAll(Vector3 quadOffset, Vector3[] ps, VisualEntity3D[] quads, double duration, float start, float end) {
    Task[] tasks = [FoldQuad(quads[1], ps[1], quadOffset + new Vector3(1.0f, 0.0f, 0.0f), Vector3.UP, start, end, duration),
    FoldQuad(quads[2], ps[2], quadOffset + new Vector3(0.0f, 1.0f, 0.0f), -Vector3.RIGHT, start, end, duration),
    FoldQuad(quads[3], ps[3], quadOffset + Vector3.ZERO, -Vector3.UP, start, end, duration),
    FoldQuad(quads[4], ps[4], quadOffset + Vector3.ZERO, Vector3.RIGHT, start, end, duration),
    FoldQuad(quads[5], ps[5], new Vector3(1.0f, 0.0f, 0.0f), Vector3.UP, start, end, duration)];
    return Task.WhenAll(tasks);
}

//Vector2[,] 

public async Task<Line3D> DrawLine(Vector3 start, Vector3 end, Color color, float width, float time = 1.0f) {
    var line = new Line3D();
    line.Color = color;
    line.Vertices = new Vector3[] { start, start };
    line.Width = width;
    world.CreateInstantly(line);
    await Animate.InterpF(x => {
        line.Vertices = new Vector3[] { start, Vector3.Lerp(start, end, x) };
    }, 0.0f, 1.0f, time);
    return line;
}

Vector2[,] faceQuadPositions = new Vector2[6, 4] {
    { new Vector2(0.0f, 0.0f), new Vector2(1.0f, 0.0f), new Vector2(1.0f, 1.0f), new Vector2(0.0f, 1.0f) },
    { new Vector2(1.0f, 0.0f), new Vector2(0.0f, 0.0f), new Vector2(0.0f, 1.0f), new Vector2(1.0f, 1.0f) },
    { new Vector2(0.0f, 1.0f), new Vector2(1.0f, 1.0f), new Vector2(1.0f, 0.0f), new Vector2(0.0f, 0.0f) },
    { new Vector2(0.0f, 0.0f), new Vector2(1.0f, 0.0f), new Vector2(1.0f, 1.0f), new Vector2(0.0f, 1.0f) },
    { new Vector2(0.0f, 0.0f), new Vector2(1.0f, 0.0f), new Vector2(1.0f, 1.0f), new Vector2(0.0f, 1.0f) },
    { new Vector2(1.0f, 0.0f), new Vector2(0.0f, 0.0f), new Vector2(0.0f, 1.0f), new Vector2(1.0f, 1.0f) },
};

Vector3[] lineQuadVertices = new Vector3[] {
    new Vector3(0.0f, 0.0f, 0.0f),
    new Vector3(1.0f, 0.0f, 0.0f),
    new Vector3(1.0f, 0.0f, 0.0f),
    new Vector3(1.0f, 1.0f, 0.0f),
    new Vector3(1.0f, 1.0f, 0.0f),
    new Vector3(0.0f, 1.0f, 0.0f),
    new Vector3(0.0f, 1.0f, 0.0f),
    new Vector3(0.0f, 0.0f, 0.0f),
};

Text2D titleText;

public async Task Intro() {
    var bigTitle = new Text2D("Cubical Marching Squares");
    bigTitle.Color = titleText.Color;
    bigTitle.Size = 64.0f;
    bigTitle.Anchor = new Vector2(0.0f, 0.2f);
    bigTitle.HAlign = TextHorizontalAlignment.Center;

    var subTitle = new Text2D("an isosurface extraction algorithm");
    subTitle.Size = bigTitle.Size * 0.5f;
    subTitle.Color = bigTitle.Color * 0.5f;
    subTitle.HAlign = TextHorizontalAlignment.Center;
    subTitle.Anchor = bigTitle.Anchor;
    subTitle.Position = new Vector2(0.0f, -80.0f);

    await Animate.CreateText(bigTitle, mode: TextCreationMode.Fade, charDelay: 0.0f);
    await Animate.CreateText(subTitle, charDelay: 1.0f/60.0f);

    await Time.WaitSeconds(1.0f);

    //var fadeOther = Animate.Color(bigTitle, titleText.Color, Color.TRANSPARENT, 0.5f);
    //await Animate.Color(subTitle, subTitle.Color, Color.TRANSPARENT, 0.5f);
    //await fadeOther;

    var blur = new CanvasBlurEffect(0.0f);
    Canvas.Default.AddEffect(blur);
    _ = Animate.Color(bigTitle, bigTitle.Color, bigTitle.Color.WithA(0.0f), 2.0f);
    _ = Animate.Color(subTitle, subTitle.Color, subTitle.Color.WithA(0.0f), 2.0f);
    await Animate.InterpT(blur.Radius, 6.0f, 2.0);
    Canvas.Default.ClearEffects();
}

public async Task Comparison() {
    await Animate.ChangeText(titleText, "");
    await Time.WaitFrame();

    var badColor = Color.RED * 0.7f;
    var goodColor = Color.GREEN * 0.8f;

    float lineHeight = 37.5f;
    var compareText = new Text2D("Does not preserve sharp features", size:32.0f);
    compareText.Color = badColor;
    compareText.Anchor = new Vector2(0.0f, 0.2f);
    compareText.Position = new Vector2(-576.0f, 0.0f);
    var compareText2 = world.Clone(compareText);
    compareText2.Text = "Ambiguous cases";
    compareText2.Position += new Vector2(0.0f, -1.0f*lineHeight);
    var compareText3 = world.Clone(compareText);
    compareText3.Text = "High triangle count";
    compareText3.Position += new Vector2(0.0f, -2.0f*lineHeight);
    var compareText4 = world.Clone(compareText);
    compareText4.Text = "Scalar data";
    compareText4.Position += new Vector2(0.0f, -3.0f*lineHeight);
    compareText4.Color = goodColor;
    var compareText5 = world.Clone(compareText);
    compareText5.Text = "One large lookup table";
    compareText5.Position += new Vector2(0.0f, -4.0f*lineHeight);
    compareText5.Color = badColor;
    var compareText6 = world.Clone(compareText);
    compareText6.Text = "No inter-cell dependency";
    compareText6.Position += new Vector2(0.0f, -5.0f*lineHeight);
    compareText6.Color = goodColor;

    var compareText11 = world.Clone(compareText);
    compareText11.Text = "Sharp features are preserved";
    compareText11.Position += new Vector2(675.0f, 0.0f);
    compareText11.Color = goodColor;
    var compareText22 = world.Clone(compareText2);
    compareText22.Text = "Solves topological ambiguities";
    compareText22.Position += new Vector2(675.0f, 0.0f);
    compareText22.Color = goodColor;
    var compareText33 = world.Clone(compareText3);
    compareText33.Text = "Crack-free multi-resolution";
    compareText33.Position += new Vector2(675.0f, 0.0f);
    compareText33.Color = goodColor;
    var compareText44 = world.Clone(compareText4);
    compareText44.Text = "Hermite data";
    compareText44.Position += new Vector2(675.0f, 0.0f);
    compareText44.Color = badColor;
    var compareText55 = world.Clone(compareText5);
    compareText55.Text = "Few small lookup tables";
    compareText55.Position += new Vector2(675.0f, 0.0f);
    compareText55.Color = goodColor;
    var compareText66 = world.Clone(compareText6);
    compareText66.Text = "No inter-cell dependency";
    compareText66.Position += new Vector2(675.0f, 0.0f);
    compareText66.Color = goodColor;

    var header1 = world.Clone(compareText);
    header1.Text = "Marching cubes";
    header1.Position += new Vector2(-22.5f, 2.0f*lineHeight);
    header1.Size = 42.0f;
    header1.Color = titleText.Color;
    var header2 = world.Clone(compareText);
    header2.Text = "Cubical marching squares";
    header2.Position += new Vector2(652.5f, 2.0f*lineHeight);
    header2.Size = 42.0f;
    header2.Color = titleText.Color;

    var outlineC = Color.WHITE;
    float charDelay = 1.0f/60.0f;
    float textDelay = 60.0f/60.0f;
    _ = Animate.CreateText(header1, charDelay: charDelay, outline: outlineC);

    await Animate.CreateText(header2, charDelay: charDelay, outline: outlineC);


    await Animate.CreateText(compareText, charDelay: charDelay, outline: outlineC);
    await Time.WaitSeconds(textDelay);
    await Animate.CreateText(compareText11, charDelay: charDelay, outline: outlineC);
    await Time.WaitSeconds(textDelay);


    await Animate.CreateText(compareText2, charDelay: charDelay, outline: outlineC);
    await Time.WaitSeconds(textDelay);
    await Animate.CreateText(compareText22, charDelay: charDelay, outline: outlineC);
    await Time.WaitSeconds(textDelay);

    await Animate.CreateText(compareText3, charDelay: charDelay, outline: outlineC);
    await Time.WaitSeconds(textDelay);
    await Animate.CreateText(compareText33, charDelay: charDelay, outline: outlineC);
    await Time.WaitSeconds(textDelay);

    await Animate.CreateText(compareText4, charDelay: charDelay, outline: outlineC);
    await Time.WaitSeconds(textDelay);
    await Animate.CreateText(compareText44, charDelay: charDelay, outline: outlineC);
    await Time.WaitSeconds(textDelay);

    await Animate.CreateText(compareText5, charDelay: charDelay, outline: outlineC);
    await Time.WaitSeconds(textDelay);
    await Animate.CreateText(compareText55, charDelay: charDelay, outline: outlineC);
    await Time.WaitSeconds(textDelay);

    _ = Animate.CreateText(compareText6, charDelay: charDelay, outline: outlineC);
    await Animate.CreateText(compareText66, charDelay: charDelay, outline: outlineC);

    await Time.WaitSeconds(2.0f);

    Vector2 explosionCenter = new Vector2(500.0f, -300.0f);
    Text2D[] allTexts = [compareText, compareText2, compareText3, compareText4, compareText11, compareText22, compareText33, compareText44, header1, header2, compareText5, compareText55, compareText66, compareText6];
    List<Task> explodeTasks = new();
    foreach(var text in allTexts) {
        var chars = text.CurrentShapes.Select(x => x.s);
        text.Disband();
        foreach(var cc in chars) {
            var outside = (cc.Position - explosionCenter).Normalized;
            var speed = new Random().NextSingle()*50.0f + 50.0f;
            explodeTasks.Add( Animate.Offset(cc, cc.Position + speed*30.0f*outside, 50.0f/speed));
            explodeTasks.Add( Animate.Rotate(cc, 3.1415f*5.0f*new Random().NextSingle(), 30.0f/speed));
        }
    }
    await Task.WhenAll(explodeTasks.ToArray());
    foreach(var text in allTexts) {
        world.Destroy(text);
    }
}

public async Task Animation(World world, Animator animator) {
    Canvas.Default.Units = CanvasUnits.Points;
    //Canvas.Default.Dpi = 128;
    Canvas.Default.Dpi = 48;

    var cam = world.ActiveCamera;
    cam.Position = new Vector3(0.0f, 0.0f, -5.0f);
    cam.ClearColor = new Color(0.035f, 0.03f, 0.05f, 1.0f);

    titleText = new Text2D("", size: 36.0f);
    titleText.Anchor = new Vector2(0.0f, 0.4f);
    titleText.Position = new Vector2(0.0f, 30.0f);
    titleText.Color = Color.WHITE * 0.6f;
    titleText.HAlign = TextHorizontalAlignment.Center;

    await Intro();

    //titleText.
    world.CreateInstantly(titleText);
    
    _ = Animate.ChangeText(titleText, "Marching cubes");

    var allEntities = world.BeginCapture();

    await Animate.OrbitAndLookAt(cam, Vector3.UP, Vector3.ZERO, -20.0f, 0.0f);

    this.world = world;
    this.animator = animator;

    var c1 = new Sphere(0.05f);
    c1.Color = Color.YELLOW;
    c1.Position = new Vector3(-2.5f, 0.0f, 0.5f);

    //var quad = new Quad();
    //quad.Vertices = (new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(1, 1, 0), new Vector3(0, 1, 0));

    Line3D[] quads = new Line3D[6];
    Vector3 quadOffset = new Vector3(-0.5f, -0.5f, 0.0f);

    (List<Sphere> l, Color c)[] cornerSpheres = new (List<Sphere>, Color)[8];
    for (int i = 0; i < 8; i++) {
        cornerSpheres[i] = (new List<Sphere>(), Color.BLACK);
    }

    void setCornerColor(int corner, Color c) {
        foreach (var s in cornerSpheres[corner].l) {
            s.Color = c;
        }
    }

    var egrid = make3DGrid(new Vector3(-0.5f - 3.0f, -0.5f - 2.0f, -1.0f - 1.0f), 1.0f, 6);
    var egridLines = new Line3D();
    egridLines.Color = (Color.WHITE*0.5f).WithA(0.2f);
    egridLines.Width = 3.0f;
    egridLines.Vertices = egrid.ToArray();
    var egridColor = egridLines.Color;
    await world.CreateFadeIn(egridLines, 1.0f);

    for (int i = 0; i < 6; i++) {
        var q = new Line3D();
        q.Vertices =  lineQuadVertices;
        q.Color = Color.BLACK;
        q.Width = 6.0f;
        q.SortKey.Value = -100;

        quads[i] = q;
        q.Position = quadOffset + CMS.quadOffsets[i];
        _ = world.CreateFadeIn(q, 0.5f);

        for (int j = 0; j < 4; j++) {
            var s = world.Clone(c1);
            s.ParentId.Value = q.Id;
            s.Position = new Vector3(faceQuadPositions[i, j], 0.0f);
            var c = cornerColors[CMS.faceCubeCorners[i, j]];
            s.Color = Color.BLACK;
            _ = world.CreateFadeIn(s, 0.5f);
            cornerSpheres[CMS.faceCubeCorners[i, j]].l.Add(s);
            cornerSpheres[CMS.faceCubeCorners[i, j]].c = c;
        }
    }

    quads[5].ParentId.Value = quads[1].Id;
    quads[5].Position = new Vector3(1.0f, 0.0f, 0.0f);

    Vector3[] ps = quads.Select(q => q.Position).ToArray();

    await FoldAll(quadOffset, ps, quads, 0.0f, 0.0f, MathF.PI/2.0f);

    Color exampleCubeColor = (0.2f*Color.ORANGE).WithA(1.0f);

    var exampleCube = new Cube();
    exampleCube.Color = exampleCubeColor;
    exampleCube.Position = new Vector3(-1.5f, 0.0f, 0.0f);
    exampleCube.Scale = new Vector3(3.0f, 3.0f, 3.0f);
    _ = world.CreateFadeIn(exampleCube, 0.5f);

    await Time.WaitSeconds(1.0f);
    await Animate.Color(exampleCube, exampleCubeColor, exampleCubeColor.WithA(0.4f), 1.0f);

    //await Animate.Move(exampleCube, new Vector3(-5.0f, 0.0f, 0.0f));
    await Animate.InterpF(x => {
        var color = Color.Lerp(Color.BLACK, 1.3f*Color.RED, x);
        setCornerColor(0, color);
        setCornerColor(3, color);
        setCornerColor(4, color);
        setCornerColor(7, color);
    }, 0.0f, 1.0f, 0.5f);

    Vector3[] intersections = new Vector3[4] {
        new Vector3(0.0f, -0.5f, -1.0f),
        new Vector3(0.0f, 0.5f, -1.0f),
        new Vector3(0.0f, 0.5f, 0.0f),
        new Vector3(0.0f, -0.5f, 0.0f),
    };

    var intersectionSpheres = new Sphere[4];
    for (int i = 0; i < 4; i++) {
        var s = world.Clone(c1);
        s.Position = intersections[i];
        s.Radius = 0.03f;
        s.Color = Color.YELLOW;
        intersectionSpheres[i] = s;
        _ = world.CreateFadeIn(s, 0.5f);
    }
    await Time.WaitSeconds(1.5f);

    await Animate.Color(exampleCube, exampleCube.Color, exampleCubeColor, 1.0f);
    await Time.WaitSeconds(0.5f);
    _ = Animate.Color(egridLines, egridLines.Color, Color.TRANSPARENT, 1.0f);
    await Animate.Color(exampleCube, exampleCube.Color, exampleCubeColor.WithA(0.0f), 1.0f);
    world.Destroy(exampleCube);

    var lines = new Line3D[5];
    lines[0] = await DrawLine(intersections[0], intersections[1], Color.YELLOW, 5.0f);
    lines[1] = await DrawLine(intersections[1], intersections[2], Color.YELLOW, 5.0f);
    lines[2] = await DrawLine(intersections[2], intersections[3], Color.YELLOW, 5.0f);
    lines[3] = await DrawLine(intersections[3], intersections[0], Color.YELLOW, 5.0f);
    lines[4] = await DrawLine(intersections[0], intersections[2], Color.YELLOW, 5.0f);

    var mesh = new Mesh();
    mesh.Outline = Color.YELLOW;
    mesh.Color = Color.YELLOW;
    mesh.Vertices = new Vector3[] {
        intersections[0],
        intersections[1],
        intersections[2],
        intersections[3],
    };
    mesh.Indices = new uint[] { 0, 1, 2, 2, 3, 0 };
    await world.CreateFadeIn(mesh, 0.5f);
    await Time.WaitSeconds(0.5f);

    await Animate.InterpF(x => {
        mesh.Outline = Color.Lerp(Color.YELLOW, Color.TRANSPARENT, x);
        mesh.Color = Color.Lerp(Color.YELLOW, Color.TRANSPARENT, x);
        foreach (var s in intersectionSpheres) {
            s.Color = Color.Lerp(Color.YELLOW, Color.TRANSPARENT, x);
        }
        foreach (var l in lines) {
            l.Color = Color.Lerp(Color.YELLOW, Color.TRANSPARENT, x);
        }
    }, 0.0f, 1.0f, 0.5f);
    await Time.WaitSeconds(0.5f);
    foreach (var line in lines) {
        world.Destroy(line);
    }
    foreach (var s in intersectionSpheres) {
        world.Destroy(s);
    }
    world.Destroy(mesh);
    await Time.WaitSeconds(0.5f);

    _ = Animate.ChangeText(titleText, "Cubical marching squares");

    await FoldAll(quadOffset, ps, quads, 1.0f, MathF.PI/2.0f, 0.0f);

    // separete quads
    foreach (var q in quads) {
        if (q == quads[0]) continue;
        var d = q.Position - quads[0].Position;
        if (q == quads[5]) d = Vector3.RIGHT;
        d.z = 0.0f;
        d = d.Normalized;
        _ = Animate.Offset(q, d * 0.5f);
    }
    await Time.WaitSeconds(1.0f);


    var intersectionLines = new Line3D[4];
    int[] parents = new int[] { 0, 2, 4, 5 };
    var ilines = new Line3D[4];
    for (int i = 0; i < 4; i++) {
        var line = new Line3D();
        line.Vertices = new Vector3[] {
            new Vector3(0.5f, 0.0f, 0.0f),
            new Vector3(0.5f, 1.0f, 0.0f),
        };
        line.ParentId.Value = quads[parents[i]].Id;
        line.Color = Color.YELLOW;
        line.Width = 5.0f;
        _ = world.CreateFadeIn(line, 0.5f);
        ilines[i] = line;
    }
    await Time.WaitSeconds(1.0f);

    // put quads back together
    List<Task> foldTasks = new ();
    foreach (var q in quads) {
        if (q == quads[0]) continue;
        var d = q.Position - quads[0].Position;
        if (q == quads[5]) d = Vector3.RIGHT;
        d.z = 0.0f;
        d = d.Normalized;
        foldTasks.Add(Animate.Offset(q, -d * 0.5f));
    }
    await Task.WhenAll(foldTasks);

    // fold quads
    await FoldAll(quadOffset, ps, quads, 1.0f, 0.0f, MathF.PI/2.0f);

    var xlines = new Line3D[4];
    var avgI = 0.25f*(intersections[0]+intersections[1]+intersections[2]+intersections[3]);
    async void drawX(int idx) {
        xlines[idx] = await DrawLine(intersections[idx], avgI, Color.YELLOW, 5.0f);
    }
    drawX(0);
    drawX(1);
    drawX(2);
    drawX(3);
    await Time.WaitSeconds(1.0f);

    mesh.Color = Color.YELLOW;
    mesh.Outline = Color.YELLOW;
    await world.CreateFadeIn(mesh, 0.5f);

    await Animate.OrbitAndLookAt(cam, Vector3.UP, Vector3.ZERO, 20.0f, 1.0f);
    await Animate.Offset(cam, new Vector3(0.0f, 0.0f, -13.0f));

    var msLine = CubicalMarchingSquares();

    // FADE OUT THE CUBE AND QUAD INS STUFF
    for (int i = 0; i < 6; i++) {
        _ = Animate.Color(quads[i], quads[i].Color, Color.TRANSPARENT, 0.5f);
    }
    for (int i = 0; i < 8; i++) {
        foreach (var sphere in cornerSpheres[i].l) {
            _ = Animate.Color(sphere, sphere.Color, Color.TRANSPARENT, 0.5f);
        }
    }
    _ = Animate.Color(mesh, mesh.Color, Color.TRANSPARENT, 0.5f);
    foreach(var line in ilines) {
        _ = Animate.Color(line, line.Color, Color.TRANSPARENT, 0.5f);
    }
    foreach(var line in lines) {
        _ = Animate.Color(line, line.Color, Color.TRANSPARENT, 0.5f);
    }
    foreach(var line in xlines) {
        _ = Animate.Color(line, line.Color, Color.TRANSPARENT, 0.5f);
    }

    _ = Animate.ChangeText(titleText, "Marching cubes mesh");

    var orbitTask =  Animate.OrbitAndLookAt(cam, Vector3.UP, Vector3.ZERO, 720.0f, 15.0f);

    await world.CreateFadeIn(msLine, 1.0f);
    await Time.WaitSeconds(5.0f);
    await Animate.InterpF(x => {
        msLine.Color = Color.Lerp(Color.YELLOW, Color.TRANSPARENT, x);
    }, 0.0f, 1.0f, 0.5f);
    world.Destroy(msLine);
    await Time.WaitSeconds(0.5f);

    _ = Animate.ChangeText(titleText, "Cubical marching squares mesh");

    var cmsMesh = DoCMS();
    var col = cmsMesh.Color;
    cmsMesh.Color = Color.TRANSPARENT;
    world.CreateInstantly(cmsMesh);
    await Animate.InterpF(x => {
        cmsMesh.Color = Color.Lerp(Color.TRANSPARENT, col, x);
        cmsMesh.Outline = Color.Lerp(Color.TRANSPARENT, Color.BLACK, x);
    }, 0.0f, 1.0f, 0.5f);

    await orbitTask;
    world.Destroy(cornerSpheres.SelectMany(x => x.l).ToArray());
    world.Destroy(quads);
    world.Destroy(mesh);
    world.Destroy(ilines);

    await Animate.InterpF(x => {
        cmsMesh.Color = Color.Lerp(col, Color.TRANSPARENT, x);
        cmsMesh.Outline = Color.Lerp(Color.BLACK, Color.TRANSPARENT, x);
    }, 0.0f, 1.0f, 1.0f);
    world.Destroy(cmsMesh);

    await Time.WaitSeconds(1.0f);

    var origin = new Vector3(-8.0f);
    int gridSize = 32;
    float cellSize = 0.5f;
    await Animate.Offset(cam, new Vector3(0.0f, 0.0f, 5.0f));

    cam.Position = new Vector3(0.0f, 2.0f, cam.Position.z);
    orbitTask = Animate.OrbitAndLookAt(cam, Vector3.UP, Vector3.ZERO, 720.0f, 20.0f);

    /*var gridVerts = make3DGrid(origin, cellSize, gridSize);
    var grid = new Line3D(mode: MeshVertexMode.Segments);
    grid.Color = new Color(0.05f, 0.05f, 0.05f, 1.0f);
    grid.Width = 2.0f;
    grid.Vertices = gridVerts.ToArray();*/

    origin = new Vector3(-2.0f);
    var grid = await AnimateGrid(origin, 1.0f, 4, new Color(0.15f, 0.05f, 0.02f, 0.35f));
    grid.Width = 1.0f;

    //await world.CreateFadeIn(grid, 1.0f);

    await Time.WaitSeconds(1.0f);
    await orbitTask;

    world.EndCapture();
    foreach (var e in allEntities) {
        if (!e.ManagedLifetime) {
            world.Destroy(e);
        }
    }

    await AnimateAmbiguous2D();

    await AnimateAmbiguous3D();

    await AnimateOctree();

    await Comparison();

    await Time.WaitSeconds(0.2f);
    var byeText = new Text2D("Thanks for watching!");
    byeText.Size = 64.0f;
    byeText.Anchor = new Vector2(0.0f, 0.2f);
    byeText.HAlign = TextHorizontalAlignment.Center;
    byeText.Color = titleText.Color;
    await Animate.CreateText(byeText);
    await Time.WaitSeconds(2.0f);
}

class VisEdgeNode {
    // for leaf nodes, s and e are the start and end points of the edge
    public Vector3 s;
    public Vector3 e;

    public (Vector3 p, Vector3 n)? intersection = null;

    public VisEdgeNode first = null;
    public VisEdgeNode second = null;

    public (VisEdgeNode, VisEdgeNode) Split() {
        var mid = (s + e) * 0.5f;
        var first = new VisEdgeNode() { s = s, e = mid };
        var second = new VisEdgeNode() { s = mid, e = e };
        return (first, second);
    }

    public bool isLeaf {
        get {
            return first == null && second == null;
        }
    }

	public (Vector3 p, Vector3 n)? getIntersection() {
		if (intersection.HasValue) {
			return intersection;
		}
		else {
			return first?.getIntersection() ?? second?.getIntersection();
		}
	}

	public (VisEdgeNode e, Vector3 p, Vector3 n)? getIntersectionWithEdge() {
		if (intersection.HasValue) {
			if (!intersection.HasValue) {
				return null;
			} else {
				return (this, intersection.Value.p, intersection.Value.n);
			}
		}
		else {
			return first?.getIntersectionWithEdge() ?? second?.getIntersectionWithEdge();
		}
	}

    public void Subdivide() {
        Debug.Assert(first == null && second == null, "Only leaf nodes can be subdivided");
        (first, second) = Split();
    }
}

class VisFaceNode {
    public VisFaceNode[] children = new VisFaceNode[4];
    public VisEdgeNode[] edges = new VisEdgeNode[4];

    public bool isLeaf {
        get {
            return children[0] == null;
        }
    }

    public void Subdivide(Vector3 min, float size, int face) {
        Debug.Assert(edges[0] != null, "Only leaf nodes can be subdivided");
        // generate the edges
        children[0] = new VisFaceNode();
        children[1] = new VisFaceNode();
        children[2] = new VisFaceNode();
        children[3] = new VisFaceNode();

        // outer edges are subdivided
        foreach (var e in edges) {
            if (e.isLeaf) {
                e.Subdivide();
            }
        }

        // inner edges are created
        VisEdgeNode[] internalEdges = new VisEdgeNode[4];
        for (int i = 0; i < 4; i++) {
            int idx1 = CMS.faceCubeCorners[face, i];
            int idx2 = CMS.faceCubeCorners[face, (i + 1) % 4];
            var s = CMS.cornerOffsets[idx1] * size + min;
            var e = CMS.cornerOffsets[idx2] * size + min;
            var v1 = (s + e) * 0.5f;
            var v2 = CMS.faceCenters[face] * size + min;
            var dif = v2 - v1;
            // edges always point towards positive direction
            bool flip = dif.x < 0.0f || dif.y < 0.0f || dif.z < 0.0f;
            internalEdges[i] = new VisEdgeNode() {
                s = flip ? v2 : v1,
                e = flip ? v1 : v2,
            };
        }

        // assign outer
		switch (face) {
			default:
        children[0].edges[0] = edges[0].first;
        children[1].edges[0] = edges[0].second;
        children[1].edges[1] = edges[1].first;
        children[2].edges[1] = edges[1].second;
        children[2].edges[2] = edges[2].second;
        children[3].edges[2] = edges[2].first;
        children[3].edges[3] = edges[3].second;
        children[0].edges[3] = edges[3].first;
		break;
			case 2:
			case 4:
        children[0].edges[0] = edges[0].first;
        children[1].edges[0] = edges[0].second;
        children[1].edges[1] = edges[1].second;
        children[2].edges[1] = edges[1].first;
        children[2].edges[2] = edges[2].second;
        children[3].edges[2] = edges[2].first;
        children[3].edges[3] = edges[3].first;
        children[0].edges[3] = edges[3].second;
		break;
			case 1:
			case 3:
        children[0].edges[0] = edges[0].second;
        children[1].edges[0] = edges[0].first;
        children[1].edges[1] = edges[1].first;
        children[2].edges[1] = edges[1].second;
        children[2].edges[2] = edges[2].first;
        children[3].edges[2] = edges[2].second;
        children[3].edges[3] = edges[3].second;
        children[0].edges[3] = edges[3].first;
		break;
		}
 
        // assign internal
		switch(face) {
		case 0:
		case 5:
        children[0].edges[1] = internalEdges[0];
        children[0].edges[2] = internalEdges[3];
        children[1].edges[3] = internalEdges[0];
        children[1].edges[2] = internalEdges[1];
        children[2].edges[0] = internalEdges[1];
        children[2].edges[3] = internalEdges[2];
        children[3].edges[0] = internalEdges[3];
        children[3].edges[1] = internalEdges[2];
		break;
		case 1:
		case 3:
		children[0].edges[1] = internalEdges[0];
        children[0].edges[2] = internalEdges[1];
        children[1].edges[3] = internalEdges[0];
        children[1].edges[2] = internalEdges[3];
        children[2].edges[0] = internalEdges[3];
        children[2].edges[3] = internalEdges[2];
        children[3].edges[0] = internalEdges[1];
        children[3].edges[1] = internalEdges[2];
		break;
		case 2:
		case 4:
        children[0].edges[1] = internalEdges[2];
        children[0].edges[2] = internalEdges[3];
        children[1].edges[3] = internalEdges[2];
        children[1].edges[2] = internalEdges[1];
        children[2].edges[0] = internalEdges[1];
        children[2].edges[3] = internalEdges[0];
        children[3].edges[0] = internalEdges[3];
        children[3].edges[1] = internalEdges[0];
		break;
		}

        foreach(var child in children) {
            foreach(var edge in child.edges) {
                Debug.Assert(edge != null);
            }
        }
    }
}

class VisNode {
    public VisNode[] children = new VisNode[8];
    public VisNode parent = null;
    public Cube cell = null;
    public int depth;
    public float size;
    public Vector3 min;
	public byte corners;

    public VisFaceNode[] faces = new VisFaceNode[6];

    // for root node
    public VisNode(Vector3 min, float size) {
        this.min = min;
        this.size = size;
        this.depth = 0;
        var edges = new VisEdgeNode[12];
        for (int i = 0; i < 12; i++) {
            var (si, ei) = CMS.cubeEdgeCorners[i];
            var s = CMS.cornerOffsets[si] * size + min;
            var e = CMS.cornerOffsets[ei] * size + min;
            edges[i] = new VisEdgeNode() { s = s, e = e };
        }
        for (int i = 0; i < 6; i++) {
            var faceNode = new VisFaceNode();
            faceNode.edges = new VisEdgeNode[4];
            for (int j = 0; j < 4; j++) {
                faceNode.edges[j] = edges[CMS.faceEdgeToCubeEdgeXX[i, j]];
            }
            this.faces[i] = faceNode;
        }
    }

    private void SanityCheck_(Dictionary<VisEdgeNode, string> edgeSet) {
        for(int i = 0; i < 6; i++) {
            var face = faces[i];
            if (!face.isLeaf) continue;
            for (int j = 0; j < 4; j++) {
                var edge = face.edges[j];
                if (!edgeSet.ContainsKey(edge)) {
                    edgeSet.Add(edge, $"{this.min} {this.size} {i} {j}");
                }
            }
        }
        foreach(var child in children) {
            child?.SanityCheck_(edgeSet);
        }
    }

    public void SanityCheck() {
        Dictionary<VisEdgeNode, string> edgeSet = new ();
        this.SanityCheck_(edgeSet);
        var elist = edgeSet.ToList();
        for (int i = 0; i < elist.Count; i++) {
            for (int j = i + 1; j < elist.Count; j++) {
                if (elist[i].Key.s == elist[j].Key.s && elist[i].Key.e == elist[j].Key.e) {
                    Debug.Assert(false, $"Duplicate edge found: {elist[i].Key.s} - {elist[i].Key.e} {elist[i].Value} {elist[j].Value}");
                }
                if (elist[i].Key.s == elist[j].Key.e && elist[i].Key.e == elist[j].Key.s) {
                    Debug.Assert(false, $"Duplicate edge found (reversed): {elist[i].Key.s} - {elist[i].Key.e}");
                }
            }
        }
    }

    // for subdivision
    private VisNode(VisNode parent, Vector3 min, float size) {
        Debug.Assert(faces.Length == 6, "Faces must have 6 elements");
        this.parent = parent;
        this.min = min;
        this.size = size;
        this.depth = parent.depth + 1;
    }

    private void SubdivideFaces() {
        for (int i = 0; i < 6; i++) {
            //face might be shared, so it might already be subdivided
            if (faces[i].isLeaf) {
                faces[i].Subdivide(min, size, i);
            }
        }
    }
    
    private void ConnectChildFaces() {
        // outer faces
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 4; j++) {
                var (childIdx, _) = CMS.faceCells[i, j];
                children[childIdx].faces[i] = faces[i].children[j];
            }
        }

        // create inner faces (without edges assigned)
        var newFaces = new VisFaceNode[12];
        for (int i = 0; i < 12; i++) {
            newFaces[i] = new VisFaceNode();
            // TODO: populate face
            var (cell1, face1) = CMS.internalFaceCells[i, 0];
            var (cell2, face2) = CMS.internalFaceCells[i, 1];
            children[cell1].faces[face1] = newFaces[i];
            children[cell2].faces[face2] = newFaces[i];
        }

        // create new edges at the center of 4 cells
        var cellCenter = min + new Vector3(size * 0.5f);
        VisEdgeNode[] newEdges = [
            new VisEdgeNode() { s = cellCenter, e = min + CMS.faceCenters[0] * size},
            new VisEdgeNode() { s = cellCenter, e = min + CMS.faceCenters[1] * size},
            new VisEdgeNode() { s = cellCenter, e = min + CMS.faceCenters[2] * size},
            new VisEdgeNode() { s = min + CMS.faceCenters[3] * size, e = cellCenter},
            new VisEdgeNode() { s = min + CMS.faceCenters[4] * size, e = cellCenter},
            new VisEdgeNode() { s = min + CMS.faceCenters[5] * size, e = cellCenter},
        ];
        // assign new edges to faces
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 4; j++) {
                var (cell, edge) = CMS.faceCells[i, j];
                var (face1, quadEdge1) = CMS.cubeEdgeToFaceEdgeXX[edge, 0];
                var (face2, quadEdge2) = CMS.cubeEdgeToFaceEdgeXX[edge, 1];
                children[cell].faces[face1].edges[quadEdge1] = newEdges[i];
                children[cell].faces[face2].edges[quadEdge2] = newEdges[i];
            }
        }

        // assign outer edges to new faces
        // yz plane
        children[0].faces[1].edges[0] = faces[4].children[3].edges[1];
        children[0].faces[1].edges[1] = faces[5].children[0].edges[1];
        children[3].faces[1].edges[1] = faces[5].children[3].edges[1];
        children[3].faces[1].edges[2] = faces[2].children[3].edges[1];
        children[4].faces[1].edges[3] = faces[0].children[0].edges[1];
        children[4].faces[1].edges[0] = faces[4].children[0].edges[1];
        children[7].faces[1].edges[2] = faces[2].children[0].edges[1];
        children[7].faces[1].edges[3] = faces[0].children[3].edges[1];
        // xy plane
        children[0].faces[0].edges[0] = faces[4].children[3].edges[0];
        children[0].faces[0].edges[3] = faces[3].children[1].edges[3];
        children[1].faces[0].edges[0] = faces[4].children[2].edges[0];
        children[1].faces[0].edges[1] = faces[1].children[1].edges[3];
        children[2].faces[0].edges[1] = faces[1].children[2].edges[3];
        children[2].faces[0].edges[2] = faces[2].children[2].edges[0];
        children[3].faces[0].edges[2] = faces[2].children[3].edges[0];
        children[3].faces[0].edges[3] = faces[3].children[2].edges[3];
        // xz plane
        children[0].faces[2].edges[3] = faces[3].children[2].edges[0];
        children[0].faces[2].edges[2] = faces[5].children[3].edges[0];
        children[1].faces[2].edges[2] = faces[5].children[2].edges[0];
        children[1].faces[2].edges[1] = faces[1].children[2].edges[0];
        children[4].faces[2].edges[3] = faces[3].children[3].edges[0];
        children[4].faces[2].edges[0] = faces[0].children[3].edges[0];
        children[5].faces[2].edges[0] = faces[0].children[2].edges[0];
        children[5].faces[2].edges[1] = faces[1].children[3].edges[0];
    }

    public async Task<IList<VisNode>> SubdivideWithLines(List<Vector3> vertices, bool quick) {
        float alph = 1.0f;
        Color[] colors = [Color.RED.WithA(alph), Color.BLUE.WithA(alph), Color.YELLOW.WithA(alph), Color.GREEN.WithA(alph)];

        var lines = CubicalMS.CellInnerEdges(this.min, this.size);
        var verts = lines.SelectMany(l => new Vector3[] { l.Item1, l.Item2 }).ToArray();
        if (true || !quick) {
            var transLine = new Line3D(mode: MeshVertexMode.Segments);
            transLine.Color = Color.VIOLET;
            transLine.Vertices = verts;
            transLine.Width = 5.0f;
            await World.current.CreateFadeIn(transLine, 0.5f);
            await Time.WaitFrame();
            World.current.Destroy(transLine);
        }

        vertices.AddRange(verts);

        SubdivideFaces();

        for (int i = 0; i < 8; i++) {
            var co = CMS.cornerOffsets[i];
            Vector3 childMin = min + co * size * 0.5f;
            var childNode = new VisNode(this, childMin, size * 0.5f);
            childNode.cell = null;
            this.children[i] = childNode;
        }

        ConnectChildFaces();

        return this.children;
    }

    public async Task<IList<VisNode>> Subdivide() {
        float alph = 1.0f;
        Color[] colors = [Color.RED.WithA(alph), Color.BLUE.WithA(alph), Color.YELLOW.WithA(alph), Color.GREEN.WithA(alph)];

        SubdivideFaces();

        World.current.Destroy(this.cell);
        List<Task> tasks = new ();
        for (int i = 0; i < 8; i++) {
            var co = CMS.cornerOffsets[i];
            Vector3 childMin = min + co * size * 0.5f;
            var acell = new Cube();
            acell.Color = colors[(depth) % colors.Length];
            acell.Scale = new Vector3(size * 0.5f);
            acell.Position = childMin + new Vector3(size * 0.25f);
            acell.Outline = Color.TRANSPARENT;
            async Task subdivideChild() {
                //await world.CreateFadeIn(acell, 1.0f)
                acell.Color = cell.Color;
                World.current.CreateInstantly(acell);
                var iscale = acell.Scale;
                var ipos = acell.Position;
                _ = Animate.Color(acell.OutlineProperty, Color.TRANSPARENT, Color.BLACK, 0.3f);
                /*_ = Animate.InterpF(x => {
                    acell.Position = Vector3.Lerp(ipos, ipos + new Vector3(0.05f), x);
                    acell.Scale = Vector3.Lerp(iscale, iscale - new Vector3(0.1f), x);
                }, 0.0f, 1.0f, 0.1f);*/
                await Animate.Color(acell, acell.Color, colors[(depth + 1) % colors.Length], 0.6f);
            }
            tasks.Add(subdivideChild());
            var childNode = new VisNode(this, childMin, size * 0.5f);
            childNode.cell = acell;
            this.children[i] = childNode;
        }
        ConnectChildFaces();
        await Task.WhenAll(tasks);
        return this.children;
    }

    public void AssignIntersections(HermiteData data) {
		// assign corners
		int step = (int)Math.Round(this.size / data.step);
		Debug.Assert(MathF.Abs(step - this.size / data.step) < 0.0001f, $"Step size should be equal to data step size. Expected: {data.step / this.size}, Actual: {step}");
		int ox = (int)Math.Floor((this.min.x - data.offset.x) / data.step);
		int oy = (int)Math.Floor((this.min.y - data.offset.y) / data.step);
		int oz = (int)Math.Floor((this.min.z - data.offset.z) / data.step);
		Debug.Assert(MathF.Abs(ox - (this.min.x - data.offset.x) / data.step) < 0.0001f, "Offset should be equal to data offset");
		Debug.Assert(MathF.Abs(oy - (this.min.y - data.offset.y) / data.step) < 0.0001f, "Offset should be equal to data offset");
		Debug.Assert(MathF.Abs(oz - (this.min.z - data.offset.z) / data.step) < 0.0001f, "Offset should be equal to data offset");
		byte corners = 0;
		for (int i = 0; i < 8; i++) {
			var offset = CMS.cornerOffsetsI[i];
			int xi = ox + offset.x * step;
			int yi = oy + offset.y * step;
			int zi = oz + offset.z * step;
			bool val = data.isInside[xi, yi, zi];
			corners |= (byte)(val ? 1 << i : 0);
		}
		this.corners = corners;

		// assign face intersections
        for (int i = 0; i < 6; i++) {
            var face = faces[i];
            if (!face.isLeaf) {
                continue;
            }

            bool[] samples = new bool[4];
            for (int j = 0; j < 4; j++) {
                int corner = CMS.faceCubeCornersXX[i, j];
                samples[j] = ((this.corners >> corner) & 1) == 1;
            }

            for (int j = 0; j < 4; j++) {
                var edge = face.edges[j];
                if (edge.intersection.HasValue) {
                    continue; // already assigned
                }
                var edgeIdx = CMS.faceEdgeToCubeEdgeXX[i, j];
                var intrs = findEdgeIntersections(data, min, size, edgeIdx);
                if (intrs.Length == 0) {
                    if (samples[j] != samples[(j+1)%4]) throw new Exception("this edge shoud have intersection!");
                    continue; // no intersection found

                }
                if (edge.isLeaf) {
                    Debug.Assert(intrs.Length <= 1, $"{intrs.Length}");
                    edge.intersection = intrs[0];
                }
            }
        }

        foreach(var child in children) {
            if (child == null) continue;
            child.AssignIntersections(data);
        }
    }

    public abstract class CMSVertex {
        public abstract override bool Equals(object obj);
        public abstract override int GetHashCode();
    }

    public class CMSEdgeVertex : CMSVertex {
        public VisEdgeNode node { get; set; }
        public CMSEdgeVertex(VisEdgeNode node) {
            this.node = node;
        }

        public override bool Equals(object obj) => obj is CMSEdgeVertex ev ? ReferenceEquals(this.node, ev.node) : false;

        public override int GetHashCode() => System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(node);
    }

    public class CMSNewVertex : CMSVertex {
        public Vector3 position { get; init; }

        public CMSNewVertex(Vector3 position)
        {
            this.position = position;
        }

        public override bool Equals(object obj) => ReferenceEquals(this, obj);

        public override int GetHashCode() => System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(this);
    }

    public record CMSSegment(CMSVertex v1, CMSVertex v2, VisFaceNode face);

    static void EvaluateFace(VisFaceNode face, int faceId, List<CMSSegment> segments, byte cornerBits, VisNode node, bool quick) {	
		float size = node.size;
		Vector3 min = node.min;

		bool[] samples = new bool[4];
		for (int i = 0; i < 4; i++) {
			int corner = CMS.faceCubeCornersXX[faceId, i];
			samples[i] = ((cornerBits >> corner) & 1) == 1;
		}

		int caseId = (samples[0] ? 1 : 0) |
			(samples[1] ? 2 : 0) |
			(samples[2] ? 4 : 0) |
			(samples[3] ? 8 : 0);

		if (caseId == 0 || caseId == 15) {
			return; // no segments to draw
		}

        Vector3? getSharpFeature(CMSEdgeVertex e1, CMSEdgeVertex e2) {
            // NOTE: always use leaf nodes
            //   this is because a parent would be considered a different node, even though they point to same intersection
            //   when detecing segment loops this is important
            //   otherwise transition cells don't connect properly
            var intr1 = e1.node.getIntersectionWithEdge();
			var intr2 = e2.node.getIntersectionWithEdge();
			Debug.Assert(intr1 != null && intr1.HasValue, $"Intersection for edge {e1} not found");
			Debug.Assert(intr2 != null && intr2.HasValue, $"Intersection for edge {e2} not found");
			e1.node = intr1 == null ? e1.node : intr1.Value.e;
			e2.node = intr2 == null ? e2.node : intr2.Value.e;
			var n1 = CMS.CubeDirToFaceDirXX(faceId, intr1.Value.n);
			var n2 = CMS.CubeDirToFaceDirXX(faceId, intr2.Value.n);
            bool haveNan = n1.ContainsNan || n2.ContainsNan;
            var t1 = n1.PerpCw;
			var t2 = n2.PerpCcw;
			var det = t1.x * t2.y - t1.y * t2.x;
            if (haveNan || float.Abs(det) < 1e-6 || Vector2.Dot(n1, n2) >= SHARP_FEATURE_ANGLE_THRESHOLD) {
                return null;
            } else {
                var p1 = CMS.CubePosToFacePosXX(faceId, intr1.Value.p - min, size);
				var p2 = CMS.CubePosToFacePosXX(faceId, intr2.Value.p - min, size);
				var t = (t2.y*(p2.x-p1.x) - t2.x*(p2.y-p1.y)) / det;
				var p3 = p1 + t * t1;
				p3.x = float.Clamp(p3.x, 0.0f, size);
				p3.y = float.Clamp(p3.y, 0.0f, size);
				var p3World = min + CMS.FacePosToCubePosXX(faceId, p3, size);
                return p3World;
            }
        }

		// ambiguous case
		if (caseId != 5 && caseId != 10) {
			// non ambiguous cases always have only 1 segment
			int startEdge = CMS.quadSegmentsXX[caseId, 0];
			int endEdge = CMS.quadSegmentsXX[caseId, 1];
			var e1 = new CMSEdgeVertex(face.edges[startEdge]);
			var e2 = new CMSEdgeVertex(face.edges[endEdge]);
            Vector3? sharpFeature = getSharpFeature(e1, e2);
			if (!sharpFeature.HasValue) {
				var seg = new CMSSegment(e1, e2, face);
				segments.Add(seg);
			} else {
				var newVert = new CMSNewVertex(sharpFeature.Value);
                if (e1.node.getIntersection().Value.p == newVert.position || e2.node.getIntersection().Value.p == newVert.position) {
                    segments.Add(new CMSSegment(e1, e2, face));
                } else {
                    segments.Add(new CMSSegment(e1, newVert, face));
                    segments.Add(new CMSSegment(newVert, e2, face));
                }
			}
		} else {
            int startEdge1 = CMS.quadSegmentsXX[caseId, 0];
			int endEdge1 = CMS.quadSegmentsXX[caseId, 1];
            int startEdge2 = CMS.quadSegmentsXX[caseId, 2];
            int endEdge2 = CMS.quadSegmentsXX[caseId, 3];
			var e1 = new CMSEdgeVertex(face.edges[startEdge1]);
			var e2 = new CMSEdgeVertex(face.edges[endEdge1]);
			var e3 = new CMSEdgeVertex(face.edges[startEdge2]);
			var e4 = new CMSEdgeVertex(face.edges[endEdge2]);
            Vector3? sharpFeature1 = getSharpFeature(e1, e2);
            Vector3? sharpFeature2 = getSharpFeature(e3, e4);
            sharpFeature1 = null;
            sharpFeature2 = null;
            if (!sharpFeature1.HasValue) {
                segments.Add(new CMSSegment(e1, e2, face));
            } else {
                var newVert = new CMSNewVertex(sharpFeature1.Value);
                if (e1.node.getIntersection().Value.p == newVert.position || e2.node.getIntersection().Value.p == newVert.position) {
                    segments.Add(new CMSSegment(e1, e2, face));
                } else {
                    segments.Add(new CMSSegment(e1, newVert, face));
                    segments.Add(new CMSSegment(newVert, e2, face));
                }
            }
            if (!sharpFeature2.HasValue) {
                segments.Add(new CMSSegment(e3, e4, face));
            } else {
                var newVert = new CMSNewVertex(sharpFeature2.Value);
                if (e3.node.getIntersection().Value.p == newVert.position || e4.node.getIntersection().Value.p == newVert.position) {
                    segments.Add(new CMSSegment(e3, e4, face));
                } else {
                    segments.Add(new CMSSegment(e3, newVert, face));
                    segments.Add(new CMSSegment(newVert, e4, face));
                }
            }
            // TODO: check if intersect, use alternative segments if do
		}
    }

    Line3D visLines;

    public async Task<Dictionary<VisFaceNode, List<CMSSegment>>> EvaluateFaces() {
        // Find all unique leaf faces
        Dictionary<VisFaceNode, (int face, VisNode node)> faceSet = new();
		Dictionary<VisFaceNode, List<CMSSegment>> faceSegments = new();

        void traverse(VisNode node) {
            for(int i = 0; i < 6; i++) {
                var face = node.faces[i];
                if (face.isLeaf) {
                    if (!faceSet.ContainsKey(face)) {
                        faceSet.Add(face, (i, node));
                    }
                }
            }
            foreach(var child in node.children) {
                if (child != null) {
                    traverse(child);
                }
            }
        }
		traverse(this);
        var faces = faceSet.ToList();

		Debug.Log($"Found {faces.Count} unique leaf faces");

		List<Vector3> vertices = new();
		visLines = new Line3D(mode: MeshVertexMode.Segments);
		visLines.Color = Color.BLUE;
        visLines.Width = 5.0f;
		World.current.CreateInstantly(visLines);
        void updateSegs(List<CMSSegment> segs) {
            foreach(var seg in segs) {
                vertices.Add(seg.v1 is CMSEdgeVertex ev1 ? ev1.node.getIntersection().Value.p : ((CMSNewVertex)seg.v1).position);
                vertices.Add(seg.v2 is CMSEdgeVertex ev2 ? ev2.node.getIntersection().Value.p : ((CMSNewVertex)seg.v2).position);
            }
            visLines.Vertices = vertices.ToArray();
        }

        var cam = World.current.ActiveCamera;
        var oPos = cam.Position;
        var oRot = cam.Rotation;
        var forward = Quaternion.AngleAxis(float.DegreesToRadians(45.0f), Vector3.UP) * Quaternion.AngleAxis(float.DegreesToRadians(45.0f), Vector3.RIGHT) * Vector3.FORWARD;

        foreach (var (face, (faceId, node)) in faces.Take(150)) {
			List<CMSSegment> segs = new();

            EvaluateFace(face, faceId, segs, node.corners, node, false);
            faceSegments.Add(face, segs);

            if (segs.Count > 0) {
                var (pos, rot) = Animate.ObservingTransform(node.min, node.min + new Vector3(node.size), forward, World.current.ActiveCamera as PerspectiveCamera, 1920.0f / 1080.0f);

                Line3D testLine = null;
                testLine = new Line3D(width: 1, mode: MeshVertexMode.Segments);
                testLine.Color = Color.MAGENTA;
                testLine.SortKey.Value = -100;
                testLine.Width = 6.0f;
                World.current.CreateInstantly(testLine);
                testLine.Vertices = new Vector3[] {
                    face.edges[0].s,
                    face.edges[0].e,
                    face.edges[1].s,
                    face.edges[1].e,
                    face.edges[2].s,
                    face.edges[2].e,
                    face.edges[3].s,
                    face.edges[3].e
                };

                var flipTable = CMS.edgeFlipTable[faceId];
                Vector3[] corners = new Vector3[4];
                corners[0] = !flipTable[0] ? face.edges[0].s : face.edges[0].e;
                corners[1] = !flipTable[0] ? face.edges[0].e : face.edges[0].s;
                corners[2] = !flipTable[2] ? face.edges[2].s : face.edges[2].e;
                corners[3] = !flipTable[2] ? face.edges[2].e : face.edges[2].s;
                bool[] samples = new bool[4];
                for (int i = 0; i < 4; i++) {
                    int corner = CMS.faceCubeCornersXX[faceId, i];
                    samples[i] = ((node.corners >> corner) & 1) == 1;
                }
                Sphere[] spheres = new Sphere[4];
                for (int i = 0; i < 4; i++) {
                    spheres[i] = new Sphere(0.05f);
                    spheres[i].Position = corners[i];
                    spheres[i].Color = samples[i] ? Color.WHITE : Color.BLACK;
                    World.current.CreateInstantly(spheres[i]);
                }

                await Animate.TransformToLimited(cam, pos, rot, 0.5f, 5.0f);

                var segLine2 = new Line3D(width: 3.0f, mode: MeshVertexMode.Segments);
                segLine2.Color = Color.ORANGE;
                segLine2.Width = 5.0f;
                segLine2.Vertices = segs.SelectMany(seg => new Vector3[] {
                    seg.v1 is CMSEdgeVertex ev1 ? ev1.node.getIntersection().Value.p : ((CMSNewVertex)seg.v1).position,
                    seg.v2 is CMSEdgeVertex ev2 ? ev2.node.getIntersection().Value.p : ((CMSNewVertex)seg.v2).position
                }).ToArray();
                segLine2.SortKey.Value = -2000;
                await World.current.CreateFadeIn(segLine2, 0.5f);
                await Animate.Color(segLine2, segLine2.Color, Color.BLUE, 0.5f);
                World.current.Destroy(segLine2);
                World.current.Destroy(testLine);
                World.current.Destroy(spheres.Where(x => x != null).ToArray());

                updateSegs(segs);
            }
        }

        await Animate.TransformToLimited(cam, oPos, oRot, 0.5f, 5.0f);

        var segLine3 = new Line3D(width: 1.0f, mode: MeshVertexMode.Segments);
        segLine3.Color = Color.ORANGE;
        segLine3.Width = 5.0f;
        List<Vector3> segLine3Vertices = new();
        List<CMSSegment> missingSegs = new();
        foreach (var (face, (faceId, node)) in faces.Skip(150)) {
            List<CMSSegment> segs = new();
            EvaluateFace(face, faceId, segs, node.corners, node, true);
            faceSegments.Add(face, segs);

            segLine3Vertices.AddRange(segs.SelectMany(seg => new Vector3[] {
                seg.v1 is CMSEdgeVertex ev1 ? ev1.node.getIntersection().Value.p : ((CMSNewVertex)seg.v1).position,
                seg.v2 is CMSEdgeVertex ev2 ? ev2.node.getIntersection().Value.p : ((CMSNewVertex)seg.v2).position
            }));
            missingSegs.AddRange(segs);
        }
        segLine3.Vertices = segLine3Vertices.ToArray();
        await World.current.CreateFadeIn(segLine3, 0.5f);
        await Animate.Color(segLine3, segLine3.Color, Color.BLUE, 0.5f);
        World.current.Destroy(segLine3);
        updateSegs(missingSegs);

		await Time.WaitSeconds(2.0f);

		return faceSegments;
    }

	public async Task<(int[], int[])> GetLoops(List<CMSSegment> segments) {
		HashSet<CMSVertex> visited = new();
		Dictionary<CMSVertex, List<CMSSegment>> vertexSegments = new();
		foreach(var seg in segments) {
			if (!vertexSegments.ContainsKey(seg.v1)) {
				vertexSegments[seg.v1] = new List<CMSSegment>();
			}
			if (!vertexSegments.ContainsKey(seg.v2)) {
				vertexSegments[seg.v2] = new List<CMSSegment>();
			}
			vertexSegments[seg.v1].Add(seg);
			vertexSegments[seg.v2].Add(seg);
		}

		async Task<List<int>> BuildLoop(CMSVertex start, HashSet<CMSSegment> usedSegments)
		{
			List<int> loop = new();
			CMSVertex current = start;
			CMSVertex previous = null;

			while (true)
			{
				visited.Add(current);
				var segs = vertexSegments[current];

				CMSSegment nextSeg = segs
					.FirstOrDefault(s => !usedSegments.Contains(s));

				if (nextSeg == null) {
					Debug.Error($"No next segment found for vertex {current}. This should not happen.");
                    string printSeg(CMSVertex v) {
                        if (v is CMSEdgeVertex ve) {
                            return $"(p{ve.node.getIntersection().Value.p} ({ve.node.GetHashCode()} {ve.node.isLeaf})";
                        } else if(v is CMSNewVertex vn) {
                            return $"NV {vn.position} {vn.GetHashCode()}";
                        } else return "??";
                    }
					Debug.Log($"All segments: {string.Join(", ", segments.Select(s => $" {printSeg(s.v1)} -TO- {printSeg(s.v2)}"))}");
                    foreach (var vtx in segments.SelectMany(x => (CMSVertex[])[x.v1, x.v2])) {
                        var sphere = new Sphere(0.15f);
                        if (vtx is CMSEdgeVertex evtx) {
                            sphere.Position = evtx.node.getIntersection().Value.p;
                            sphere.Color = Color.MAGENTA;
                        } else if (vtx is CMSNewVertex nvtx) {
                            sphere.Position = nvtx.position;
                            sphere.Color = Color.CYAN;
                        }
                        World.current.CreateInstantly(sphere);
                    }
                    for (int i = 0; i < segments.Count; i++) {
                        var v1 = segments[i].v2;
                        var v2 = segments[(i+1)%segments.Count].v1;
                        Debug.Log($"{v1.Equals(v2)} {v1.GetHashCode()} {v2.GetHashCode()}");
                    }
					await Time.WaitSeconds(1700.0f);
					break; // dead end or loop complete
				}

				usedSegments.Add(nextSeg);
				loop.Add(segments.IndexOf(nextSeg));

				CMSVertex next = nextSeg.v1.Equals(current) ? nextSeg.v2 : nextSeg.v1;

				if (next.Equals(start))
				{
					// closed loop
					break;
				}

				previous = current;
				current = next;
			}

			return loop;
		}
		(int[], int[]) ret = (null, null);
		HashSet<CMSSegment> usedSegments = new();

		foreach (var v in vertexSegments.Keys)
		{
			if (!visited.Contains(v))
			{
				var loop = await BuildLoop(v, usedSegments);
				if (loop.Count > 0 && ret.Item1 == null)
				{
					ret = (loop.ToArray(), null);
				}
				else if (loop.Count > 0 && ret.Item2 == null)
				{
					ret = (ret.Item1, loop.ToArray());
					break;
				}
				else
				{
					Debug.Assert(false, "Found more than 2 loops, this is not valid.");
				}
			}
		}
        if (usedSegments.Count < segments.Count) {
            Debug.Error($"used: {usedSegments.Count} / {segments.Count}");
            //await Time.WaitSeconds(1800.0f);
        }
		return ret;
	}

	public async Task<Mesh> ExtractSurface(Text2D titleText) {
		List<CMSSegment> currentSegments = new();
		var faceSegments = await EvaluateFaces();
		Dictionary<CMSVertex, uint> vertexMap = new();
		List<(Vector3 p, Vector3? n)> vertices = new();
		List<uint> indices = new();
		uint vIdx = 0;

        var testMesh = new Mesh();
        testMesh.Color = Color.BROWN;
        testMesh.Outline = Color.BLACK;
        testMesh.SortKey.Value = -1000;
        World.current.CreateInstantly(testMesh);

        _ = Animate.ChangeText(titleText, "CMS step 5: extract surface");

		(Vector3, int)? processLoop(int[] loop, VisNode node) {
			if (loop == null) return null;
			Vector3 massPoint = Vector3.ZERO;
			float mass = 0.0f;
			List<uint> curIndices = new();
			// for sharp feature detection
			List<Vector3?> normals = new();
			void newVert(CMSVertex v) {
				if (!vertexMap.ContainsKey(v)) {
					if (v is CMSEdgeVertex ev) {
						var intr = ev.node.getIntersection();
                        if (intr.Value.p.ContainsNan)
                        {
                            throw new Exception($"NAN in EdgeVertex {node.min} {node.size}");
                        }
						vertices.Add((intr.Value.p, intr.Value.n));
					} else if(v is CMSNewVertex nv) {
						vertices.Add((nv.position, null));
                        if (nv.position.ContainsNan)
                        {
                            throw new Exception($"NAN in NewVertex {node.min} {node.size}");
                        }
					} else throw new NotImplementedException();
					massPoint += vertices.Last().p;
					vertexMap[v] = vIdx;	
					curIndices.Add(vIdx++);
				} else {
					curIndices.Add(vertexMap[v]);
					massPoint += vertices[(int)vertexMap[v]].p;
				}
			}
			foreach(var idx in loop) {
				newVert(currentSegments[idx].v1);
				newVert(currentSegments[idx].v2);
				mass += 2.0f;
			}

			float minCos = 1.0f;
			for (int i = 0; i < curIndices.Count; i++) {
				for (int j = i + 1; j < curIndices.Count; j++) {
					var v1 = vertices[(int)curIndices[i]];
					var v2 = vertices[(int)curIndices[j]];
					if (v1.n == null || v2.n == null) continue;
					var ccos = Vector3.Dot(v1.n.Value, v2.n.Value);
					if (ccos < minCos) {
						minCos = ccos;
					}
				}	
			}

			// calculate mass point
			massPoint *= 1.0f / mass;
			Vector3 newVertex;
            int sharpCase = 0;
			if (minCos > 0.9f) { // no sharp feature
				newVertex = massPoint;
                sharpCase = 1;
			} else {
				// calculate sharp feature
				var validVertices = curIndices
					.Select(i => vertices[(int)i])
					.Where(v => v.n.HasValue)
					.ToList();
				var matrix = Matrix<float>.Build.Dense(validVertices.Count, 3);
				var vector = Vector<float>.Build.Dense(validVertices.Count, 1.0f);
				for (int i = 0; i < validVertices.Count; i++) {
					var n = validVertices[i].n.Value;
					matrix.SetRow(i, new float[] { n.x, n.y, n.z });
					vector[i] = Vector3.Dot(n, validVertices[i].p - massPoint);
				}
				var svd = matrix.Svd(true);
				var mw = svd.W;
				var ms = svd.S;
				for (int i = 0; i < ms.Count; i++)
				{
					ms[i] = MathF.Abs(ms[i]) < float.Epsilon ? 0 : 1/ms[i];
				}
				mw.SetDiagonal(ms);
				var pseudoInverse = (svd.U * mw * svd.VT).Transpose();
				var result = pseudoInverse * vector;
				var p = new Vector3(result[0], result[1], result[2]) + massPoint;
				if (p.x < node.min.x || p.x > node.min.x + node.size ||
					p.y < node.min.y || p.y > node.min.y + node.size ||
					p.z < node.min.z || p.z > node.min.z + node.size) {
					newVertex = massPoint;
                    sharpCase = 3;
				} else {
					newVertex = p;
                    sharpCase = 2;
				}
			}
			uint mpIdx = vIdx++;
			vertices.Add((newVertex, null));
			for (int i = 0; i < curIndices.Count; i += 2) {
				indices.Add(curIndices[i]);
				indices.Add(curIndices[i + 1]);
				indices.Add(mpIdx);
			}
            return (newVertex, sharpCase);
		}

		List<Vector3> testLines = new();
        List<Color> testColors = new();
		var testLine = new Line3D(mode: MeshVertexMode.Segments);
		testLine.Width = 6.1f;
		testLine.Color = Color.ORANGE;
        testLine.SortKey.Value = -100;
		World.current.CreateInstantly(testLine);
		
		List<Vector3> testLines2 = new();
		List<Color> testColors2 = new();
		var testLine2 = new Line3D(mode: MeshVertexMode.Segments);
		testLine2.Width = 6.1f;
        testLine2.SortKey.Value = -50;
		//testLine2.Color = Color.CYAN;
		World.current.CreateInstantly(testLine2);

        int doneCount = 0;
        bool done = false;
        var cam = World.current.ActiveCamera as PerspectiveCamera;
        var oPos = cam.Position;
        var oRot = cam.Rotation;

		async Task recurseCells(VisNode node) {
			if (node.children[0] == null) { // leaf node
				currentSegments.Clear();
				node.GetSegments(currentSegments, faceSegments);

				testLines2.Clear();
				testColors2.Clear();
				foreach(var seg in currentSegments) {
					var p1 = seg.v1 is CMSEdgeVertex ev1 ? ev1.node.getIntersection().Value.p : ((CMSNewVertex)seg.v1).position;
					var p2 = seg.v2 is CMSEdgeVertex ev2 ? ev2.node.getIntersection().Value.p : ((CMSNewVertex)seg.v2).position;
				}
				for(int i = 0; i < 12; i++) {
					var (si, ei) = CMS.cubeEdgeCorners[i];
					var s = CMS.cornerOffsets[si] * node.size + node.min;
					var e = CMS.cornerOffsets[ei] * node.size + node.min;
					testLines2.Add(s);
					testLines2.Add(e);
					testColors2.Add(Color.RED);
					testColors2.Add(Color.RED);
				}
				List<Sphere> spheres = new();
				for (int i = 0; i < 8; i++) {
					var offset = CMS.cornerOffsets[i];
					var sphere = new Sphere(0.05f);
					sphere.Position = node.min + offset * node.size;
					sphere.Color = ((node.corners>>i)&1) == 1 ? Color.WHITE : Color.BLACK;
					World.current.CreateInstantly(sphere);
					spheres.Add(sphere);
				}
				testLine2.Vertices = testLines2.ToArray();
				testLine2.Colors = testColors2.ToArray();


                var forward = Quaternion.AngleAxis(float.DegreesToRadians(45.0f), Vector3.UP) * Quaternion.AngleAxis(float.DegreesToRadians(45.0f), Vector3.RIGHT) * Vector3.FORWARD;
                var (pos, rot) = Animate.ObservingTransform(node.min, node.min+ new Vector3(node.size), forward, World.current.ActiveCamera as PerspectiveCamera, 1920.0f/1080.0f);

				var (loop1, loop2) = await GetLoops(currentSegments);


                if (doneCount >= 10 && !done) {
                    await Animate.TransformToLimited(World.current.ActiveCamera, oPos, oRot, 0.5f, 5.0f);
                    done = true;
                }

				var sharp1 = processLoop(loop1, node);
                if (sharp1.HasValue) {
                    var sharpSphere = new Sphere(0.03f);
                    sharpSphere.Position = sharp1.Value.Item1;
                    sharpSphere.Color = sharp1.Value.Item2 switch {
                        /*0 => Color.BLACK,
                        1 => Color.VIOLET,
                        2 => Color.YELLOW,
                        3 => Color.RED,*/
                        _ => Color.YELLOW,
                    };
                    World.current.CreateInstantly(sharpSphere);
                    spheres.Add(sharpSphere);
                }
				var sharp2 = processLoop(loop2, node);
                if (sharp2.HasValue) {
                    var sharpSphere = new Sphere(0.03f);
                    sharpSphere.Position = sharp2.Value.Item1;
                    sharpSphere.Color = sharp2.Value.Item2 switch {
                        /*0 => Color.BLACK,
                        1 => Color.VIOLET,
                        2 => Color.YELLOW,
                        3 => Color.RED,*/
                        _ => Color.YELLOW,
                    };
                    World.current.CreateInstantly(sharpSphere);
                    spheres.Add(sharpSphere);
                }

				if (loop1 != null) {
					testLines.Clear();
                    testColors.Clear();
					foreach(var idx in loop1) {
						testLines.Add(currentSegments[idx].v1 is CMSEdgeVertex ev ? ev.node.getIntersection().Value.p : ((CMSNewVertex)currentSegments[idx].v1).position);
						testLines.Add(currentSegments[idx].v2 is CMSEdgeVertex ev2 ? ev2.node.getIntersection().Value.p : ((CMSNewVertex)currentSegments[idx].v2).position);
                        testColors.Add(Color.ORANGE);
                        testColors.Add(Color.ORANGE);
					}
					testLine.Vertices = testLines.ToArray();
                    testLine.Colors = testColors.ToArray();
                }
				if (loop2 != null) {
					testLines.Clear();
                    testColors.Clear();
					foreach(var idx in loop2) {
						testLines.Add(currentSegments[idx].v1 is CMSEdgeVertex ev ? ev.node.getIntersection().Value.p : ((CMSNewVertex)currentSegments[idx].v1).position);
						testLines.Add(currentSegments[idx].v2 is CMSEdgeVertex ev2 ? ev2.node.getIntersection().Value.p : ((CMSNewVertex)currentSegments[idx].v2).position);
                        testColors.Add(Color.VIOLET);
                        testColors.Add(Color.VIOLET);
					}
					testLine.Vertices = testLines.ToArray();
                    testLine.Colors = testColors.ToArray();
                
				}

                if (loop1 != null || loop2 != null) {
                    if (doneCount < 10) {
                        await Animate.TransformToLimited(World.current.ActiveCamera, pos, rot, 0.5f, 5.0f);
                        await Time.WaitSeconds(1.0f);
                    } else {
                        await Time.WaitFrame();
                    }
                    doneCount++;
                }

				currentSegments.Clear();

                testMesh.Vertices = vertices.Select(x => x.p).ToArray();
                testMesh.Indices = indices.ToArray();
                
				World.current.Destroy(spheres.ToArray());
			} else {
				foreach(var child in node.children) {
					await recurseCells(child);
				}
			}
		}
		await recurseCells(this);

		Debug.Assert(vertexMap.Values.Select(x => x).Distinct().Count() == vertexMap.Count, "Vertex indices should be unique");
        await Animate.Color(testMesh, testMesh.Color, testMesh.Color.WithA(1.0f), 0.5f);
        World.current.Destroy(testLine);
        World.current.Destroy(testLine2);

        await Animate.Color(visLines, visLines.Color, Color.TRANSPARENT, 0.5f);
        World.current.Destroy(visLines);

        return testMesh;
	}

    public void GetSegments(List<CMSSegment> outSegments, Dictionary<VisFaceNode, List<CMSSegment>> faceSegments) {
		void recurseFace(VisFaceNode face) {
			if (face.isLeaf) {
				faceSegments.TryGetValue(face, out var segments);
				outSegments.AddRange(segments);
			} else {
				foreach(var child in face.children) {
					recurseFace(child);
				}
			}
		}
		foreach(var face in faces) {
			recurseFace(face);
		}
    }
}

static (Vector3 p, Vector3 n)[] findEdgeIntersections(HermiteData data, Vector3 min, float size, int edge) {
    List<(Vector3, Vector3)> ret = new ();
	var edgeDir = CMS.cubeEdgeDir[edge];
    var (si, ei) = CMS.cubeEdgeCorners[edge];
    var s = min + CMS.cornerOffsets[si] * size;
    var e = min + CMS.cornerOffsets[ei] * size;
	var sOfst = s - data.offset;
	var step = data.step;
	int xi = (int)Math.Round(sOfst.x / step);
	int yi = (int)Math.Round(sOfst.y / step);
	int zi = (int)Math.Round(sOfst.z / step);
	float runLength = (e - s).Length;
	int runCount = (int)Math.Round(Math.Abs(runLength) / step);
	Debug.Assert(Math.Abs(runLength) / step - runCount < 0.0001f, $"Run count should be exact {Math.Abs(runLength) / step - runCount} size {size}");
	List<(Vector3, Vector3)> intersectionsList = new();
	for (int i = 0; i < runCount; i++) {
		(Vector3 v, Vector3 n)? cur = null;
		try {
		switch (edgeDir) {
			case 0:
				cur = data.intersections[edgeDir, yi, zi, xi + i];
				break;
			case 1:
				cur = data.intersections[edgeDir, xi, zi, yi + i];
				break;
			case 2:
				cur = data.intersections[edgeDir, xi, yi, zi + i];
				break;
		}
		} catch (Exception){}
		if (cur.HasValue) {
			intersectionsList.Add(cur.Value);
		}
	}
	return intersectionsList.ToArray();
}

static (Vector3 p, Vector3 n)[] findEdgeIntersections(List<(Vector3 v, Vector3 n, int dir)> intersections, Vector3 min, float size, int edge) {
    List<(Vector3, Vector3)> ret = new ();
	var edgeDir = CMS.cubeEdgeDir[edge];
    var (si, ei) = CMS.cubeEdgeCorners[edge];
    var s = min + CMS.cornerOffsets[si] * size;
    var e = min + CMS.cornerOffsets[ei] * size;
    foreach(var intr in intersections) {
		if (intr.dir != edgeDir) continue;
        var p = AMath.ClosestPointOnLineSegment(intr.v, s, e);
        var dif = p - intr.v;
        var sqdist = Vector3.Dot(dif, dif);
        if (sqdist < 0.0001f) {
            ret.Add((p, intr.n));
        }
    }
    return ret.ToArray();
}

public class HermiteData {
	public (Vector3 p, Vector3 n)?[,,,] intersections;
	public bool[,,] isInside;
	public Vector3 offset;
	public float step;

	public HermiteData(Func<Vector3, float> eval, Func<Vector3, Vector3> evalNormal, Vector3 offset, float step, int gridSize) {
		this.offset = offset;
		this.step = step;

		var gp1 = gridSize+1;
		isInside = new bool[gp1, gp1, gp1];
		intersections = new (Vector3, Vector3)?[3, gp1, gp1, gridSize];
		// eval grid
		for (int i = 0; i < gp1; i++)
		for (int j = 0; j < gp1; j++)
		for (int k = 0; k < gp1; k++) {
			var location = new Vector3(offset.x + i * step, offset.y + j * step, offset.z + k * step);
			isInside[i, j, k] = eval(location) <= 0.0f;
		}

		for (int dir = 0; dir < 3; dir++) {
			for (int k = 0; k < gridSize; k++)
			for (int i = 0; i < gp1; i++) {
				for (int j = 0; j < gp1; j++) {
					var sample1 = dir switch {
						0 => isInside[k, i, j],
						1 => isInside[i, k, j],
						2 => isInside[i, j, k],
						_ => false
					};
					var sample2 = dir switch {
						0 => isInside[k+1, i, j],
						1 => isInside[i, k+1, j],
						2 => isInside[i, j, k+1],
						_ => false
					};
					if (sample1 == sample2) continue;
					var start = dir switch {
						0 => new Vector3(offset.x + k * step, offset.y + i * step, offset.z + j * step),
						1 => new Vector3(offset.x + i * step, offset.y + k * step, offset.z + j * step),
						2 => new Vector3(offset.x + i * step, offset.y + j * step, offset.z + k * step),
						_ => Vector3.ZERO
					};
					var end = start + (dir switch {
						0 => new Vector3(step, 0.0f, 0.0f),
						1 => new Vector3(0.0f, step, 0.0f),
						2 => new Vector3(0.0f, 0.0f, step),
						_ => Vector3.ZERO
					});
					float curMin = float.MaxValue;
					Vector3 minPos = Vector3.ZERO;
					for (int s = 0; s <= 100; s++) {
						Vector3 pos = Vector3.Lerp(start, end, s / 100.0f);
						float val = eval(pos);
						float dist = val*val;
						if (dist < curMin) {
							curMin = dist;
							minPos = pos;
						}
					}
					Vector3 minNormal = evalNormal(minPos);
					intersections[dir, i, j, k] = (minPos, minNormal);
				}
			}
		}
	}
}

class SDF {
    float[,] sdf;
    Rect rect;
    float thickness;

    public SDF(float[,] sdf, Rect rect, float thickness) {
        this.sdf = sdf;
        this.rect = rect;
        this.thickness = thickness;
    }

    public float sample(Vector3 queryPos) {
        Vector2 clampedPos = new Vector2(
            Math.Clamp(queryPos.x, rect.x, rect.x + rect.width),
            Math.Clamp(queryPos.y, rect.y, rect.y + rect.height)
        );
        float imgX = ((clampedPos.x - rect.x) / rect.width) * sdf.GetLength(1);
        float imgY = (1.0f - ((clampedPos.y - rect.y) / rect.height)) * sdf.GetLength(0);
        
        int x1 = int.Clamp((int)float.Floor(imgX), 0, sdf.GetLength(1)-1);
        int y2 = int.Clamp((int)float.Floor(imgY), 0, sdf.GetLength(0)-1);
        int x2 = int.Clamp((int)float.Ceiling(imgX), 0, sdf.GetLength(1)-1);
        int y1 = int.Clamp((int)float.Ceiling(imgY), 0, sdf.GetLength(0)-1);

        var q11 = sdf[y1, x1];
        var q12 = sdf[y2, x1];
        var q21 = sdf[y1, x2];
        var q22 = sdf[y2, x2];

        float result;
        if (x1 != x2 && y1 != y2) {
            result = (1.0f / ((x2 - x1)*(y2 - y1))) * (q11*(x2-imgX)*(y2-imgY) + q12*(x2-imgX)*(imgY-y1) + q21*(imgX-x1)*(y2-imgY) + q22*(imgX - x1)*(imgY - y1));
        } else if (x1 == x2 && y1 == y2) {
            result = q11;
        } else if (x1 == x2) {
            result = (1.0f / (y2 - y1)) * (q11 * (y2 - imgY) + q12 * (imgY - y1));
        } else {
            result = (1.0f / (x2 - x1)) * (q11 * (x2 - imgX) + q21 * (imgX - x1));
        }

        float extraDist2D = (queryPos.xy - clampedPos).Length;
        float dist2D = extraDist2D + result;


        return float.Max(dist2D, float.Abs(queryPos.z) - (thickness/2.0f));
    }
}

async Task AnimateOctree() {
    var offset = new Vector3(-8.0f);
    int gCount = 16;
    float gSize = 1.0f;

    var cam = world.ActiveCamera;
    cam.Position = new Vector3(0.0f, 0.0f, -30.0f);
    cam.Rotation = Quaternion.LookRotation(Vector3.FORWARD, Vector3.UP);
    var orbitTask = Animate.OrbitAndLookAt(cam, Vector3.UP, Vector3.ZERO, 720.0f, 10.0f); 

    _ = Animate.ChangeText(titleText, "CMS step 1: hermite data");

    var grid = make3DGrid(offset, gSize, gCount);
    var gridLines = new Line3D();
    gridLines.Color = new Color(0.05f, 0.05f, 0.05f, 0.5f);
    gridLines.Width = 5.0f;
    gridLines.Vertices = grid.ToArray();
    //await world.CreateFadeIn(gridLines, 1.0f);

    await Time.WaitSeconds(3.0f);

    Func<(Vector3 s, Vector3 e), HermiteIntersection[]> getIntr = (pos) => {
        return GetIntersections(pos, Evaluate, EvaluateNormal);
    };

    var sw = new System.Diagnostics.Stopwatch();
    sw.Start();

    using var fs = System.IO.File.OpenRead("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc");
    animator.LoadFont(fs, "notofont");
    var shapes = animator.ShapeText("忍", Vector2.ZERO, 24, font: "notofont");
    //var shapes = animator.ShapeText("a", Vector2.ZERO, 24, font: "notofont");
    var mPath = shapes[0].s.Path;
    var (sdf, rect) = ShapePath.CalculateSDF(mPath, 512, 32, linearizationSteps: 100);

    var shapes2 = animator.ShapeText("愛", Vector2.ZERO, 24, font: "notofont");
    var mPath2 = shapes2[0].s.Path;
    var (sdf2, rect2) = ShapePath.CalculateSDF(mPath2, 512, 32, linearizationSteps: 100);
    Debug.Log($"Generating SDF took {sw.Elapsed.TotalSeconds} seconds");
    sw.Reset();
    sw.Start();

    var mySdf = new SDF(sdf, rect, 19.2f);
    var mySdf2 = new SDF(sdf2, rect2, 19.2f);

    var myFunc = (Vector3 pos) => float.Lerp(mySdf.sample((pos-offset)*2.5f + new Vector3(rect.x - 8.0f, rect.y - 10.0f, -10.0f)), mySdf2.sample((pos-offset)*2.5f + new Vector3(rect.x - 8.0f, rect.y - 10.0f, -10.0f)), float.Clamp((pos.z+8f)/5.0f, 0.0f, 1.0f));
    //var myFunc = (Vector3 pos) => mySdf.sample((pos-offset)*2.5f + new Vector3(rect.x - 8.0f, rect.y - 10.0f, -10.0f));

    var myFuncLast = (Vector3 pos) => opSubtract(sdSphere(pos-new Vector3(3.0f), 8.0f) ,opSubtract(mySdf.sample((pos-offset)*2.5f + new Vector3(rect.x - 8.0f, rect.y - 10.0f, -0.5f)), sdBox(pos-new Vector3(0.0f, 1.0f, 0.0f), new Vector3(6.11111111111f, 5.111111111f, 6.1111111111111f))));
    var myFuncLast2 = (System.Numerics.Vector3 pos) => myFuncLast(pos);
    Func<System.Numerics.Vector3, System.Numerics.Vector3> myFuncLast2Normal = (pos) => CalcNormal(myFuncLast, pos, eps: 0.0033f);

    var myFunc2 = (Vector3 pos) => opSubtract(myFunc(pos), Evaluate(pos));
	//var hermiteData = new HermiteData(myFunc, (pos) => CalcNormal(myFunc, pos, eps: 0.0033f), offset, gSize, gCount);
    var hermiteData = new HermiteData(Evaluate, EvaluateNormal, offset, gSize, gCount);
    int hermiteDepth = 4;
    Debug.Log($"Generating hermite data took {sw.Elapsed.TotalMilliseconds} ms");


    var myFunc3 = (System.Numerics.Vector3 pos) => myFunc(pos);

    Func<System.Numerics.Vector3, System.Numerics.Vector3> myFunc3Normal = (pos) => CalcNormal(myFunc, pos, eps: 0.0033f);
    var hermiteData2 = MyCMS.HermiteData.FromSdf(myFunc3, myFunc3Normal, offset, gSize*0.25f, 4*gCount);

    sw.Reset();
    sw.Start();
    (System.Numerics.Vector3[] vertices, int[] indices) jmesh = ([], []);
    MyCMS.CMSTree fastTree;
    int triCount = 0;
    for (int i = 0; i < 10; i++) {
        fastTree = new MyCMS.CMSTree(hermiteData2);
        fastTree.Subdivide(2);
        jmesh = fastTree.ExtractSurface();
        triCount = jmesh.indices.Length / 3;
    }
    Console.WriteLine($"Generating NewCMS mesh took {sw.Elapsed.TotalMilliseconds} ms, {triCount} triangles");

    /*
    var herms = new Line3D(mode: MeshVertexMode.Segments);
    herms.Color = Color.RED;
    List<Vector3> hermList = new();
    List<Color> hermColors = new();
    for (int i = 0; i < gCount*4; i++)
    for (int j = 0; j < gCount*4; j++) 
    for (int k = 0; k < gCount*4; k++) {
        if (true) {
        var xher = hermiteData.intersections[0, i, j, k];
        if (xher.HasValue) {
            hermList.Add(xher.Value.p);
            hermList.Add(xher.Value.p + xher.Value.n*0.1f);
            hermColors.Add(Color.RED);
            hermColors.Add(Color.RED);
        }
        var yher = hermiteData.intersections[1, i, j, k];
        if (yher.HasValue) {
            hermList.Add(yher.Value.p);
            hermList.Add(yher.Value.p + yher.Value.n*0.1f);
            hermColors.Add(Color.GREEN);
            hermColors.Add(Color.GREEN);
        }

        var zher = hermiteData.intersections[2, i, j, k];
        if (zher.HasValue) {
            hermList.Add(zher.Value.p);
            hermList.Add(zher.Value.p + zher.Value.n*0.1f);
            hermColors.Add(Color.BLUE);
            hermColors.Add(Color.BLUE);
        }
        } else {
            if (k*gSize*0.25f < 4.0f || k*gSize*0.25f > 4.1f) continue;
            var p = new Vector3(i*gSize*0.25f, j*gSize*0.25f, k*gSize*0.25f);
            var hval = hermiteData.signs[i,j,k];
            hermList.Add(p);
            hermList.Add(p + Vector3.UP*0.1f);
            hermColors.Add(hval ? Color.WHITE : Color.BLACK);
            hermColors.Add(hval ? Color.WHITE : Color.BLACK);
        }
    }
    herms.Vertices = hermList.ToArray();
    herms.Colors = hermColors.ToArray();
    world.CreateInstantly(herms);
    await Time.WaitSeconds(2000.0f);
    */

    Vector3 getOffset(int dir, int i, int j, int k) {
        return dir switch {
            0 => new Vector3(k*gSize, i*gSize, j*gSize) + offset,
            1 => new Vector3(i*gSize, k*gSize, j*gSize) + offset,
            2 => new Vector3(i*gSize, j*gSize, k*gSize) + offset,
            _ => Vector3.ZERO
        };
    }

    var iSpheres = new List<Sphere>();

    // find all intersections
    var intersections = new (Vector3 v, Vector3 n)?[3, gCount+1, gCount+1, gCount+1];
    var tempLine = new Line3D();
    tempLine.Width = 5.0f;
    tempLine.Color = Color.YELLOW;
    world.CreateInstantly(tempLine);

    var normalLines = new Line3D(); 
    normalLines.Width = 5.0f;
    normalLines.Color = Color.GREEN;
    world.CreateInstantly(normalLines);

    List<Vector3> normalLineVerts = new List<Vector3>();
	List<Color> normalLineColors = new List<Color>();
    int oldNormalCount = 0;

    List<Vector3> lineVerts = new List<Vector3>();
    for (int dir = 0; dir < 3; dir++) {
        for (int k = 0; k < gCount; k++) {
        lineVerts.Clear();

        for (int i = 0; i < gCount+1; i++) {
        for (int j = 0; j < gCount+1; j++) {
            var start = getOffset(dir, i, j, k);
            var end = start + gSize * dir switch {
                0 => new Vector3(1.0f, 0.0f, 0.0f),
                1 => new Vector3(0.0f, 1.0f, 0.0f),
                2 => new Vector3(0.0f, 0.0f, 1.0f),
                _ => Vector3.ZERO
            };
            lineVerts.AddRange(new Vector3[] { start, end });

            var intr = getIntr((start, end));
            if (intr.Length > 0) {
                intersections[dir, i, j, k] = (intr[0].pos, intr[0].normal);

                var s = new Sphere(0.05f);
                s.Color = Color.YELLOW;
                s.Position = start;
                iSpheres.Add(s);
                //world.CreateInstantly(s);

                normalLineVerts.AddRange(new Vector3[] { intr[0].pos, intr[0].pos + (dir switch { 0 => 0.3f, 1 => 0.4f, 2 => 0.5f, _ => 0.0f })*intr[0].normal });
				normalLineColors.Add(dir switch {
					0 => Color.GREEN,
					1 => Color.GREEN,
					2 => Color.GREEN,
					_ => Color.WHITE
				});
				normalLineColors.Add(normalLineColors.Last());
            } else {
                intersections[dir, i, j, k] = null;
                tempLine.Color = Color.RED;
            }

        }
        }
            if (normalLineVerts.Count > oldNormalCount) {
                normalLines.Vertices = normalLineVerts.ToArray();
				//normalLines.Colors = normalLineColors.ToArray();
                oldNormalCount = normalLineVerts.Count;
            }
            tempLine.Vertices = lineVerts.ToArray();
            await Time.WaitSeconds(0.1f);
        }
    }
    world.Destroy(tempLine);

    await orbitTask;

    await Time.WaitSeconds(1.0f);

    // destroy grid
    await Animate.Color(gridLines, gridLines.Color, Color.TRANSPARENT, 1.0f);
    world.Destroy(gridLines);

    await Time.WaitSeconds(1.0f);
    // create octree

    async Task<VisNode> SubdivideBfs(Vector3 parentMin, float size, int maxDepth) {
        Queue<(Vector3 min, float size, int depth, VisNode parent)> queue = new ();

        float alph = 1.0f;
        Color[] colors = [Color.RED.WithA(alph), Color.BLUE.WithA(alph), Color.YELLOW.WithA(alph), Color.GREEN.WithA(alph)];

        var rootCell = new Cube();
        rootCell.Color = colors[0];
        rootCell.Scale = new Vector3(size);
        rootCell.Position = parentMin + new Vector3(size * 0.5f);
        rootCell.Outline = Color.BLACK;
        await world.CreateFadeIn(rootCell, 0.5f);
        await Time.WaitSeconds(0.25f);
        var node = new VisNode(parentMin, size);
        node.cell = rootCell;

        queue.Enqueue((parentMin, size, 0, node));

        while (queue.Count > 0) {
            var (min, s, d, p) = queue.Dequeue();
            if (d >= maxDepth) continue;
            
            var nodes = await p.Subdivide();
            for (int i = 0; i < 8; i++) {
                var co = CMS.cornerOffsets[i];
                Vector3 childMin = min + co * s * 0.5f;
                queue.Enqueue((childMin, s * 0.5f, d + 1, nodes[i]));
            }
            if (p != null) {
                world.Destroy(p.cell); // destroy parent cell
            }
        }
        return node;
    }

    _ = Animate.ChangeText(titleText, "CMS step 2: initialize octree that intersects geometry");

    //await Subdivide(new Vector3(-8.0f), 16.0f, 0, 2);
    _ = Animate.OrbitAndLookAt(cam, Vector3.UP, Vector3.ZERO, -405.0f, 3.0f);
    var tree = await SubdivideBfs(new Vector3(-8.0f), 16.0f, 2);
    List<VisNode> leaves = new List<VisNode>();
    void getLeaves(VisNode node) {
        if (node.children.All(c => c == null)) {
            leaves.Add(node);
        } else {
            foreach (var child in node.children) {
                if (child != null) getLeaves(child);
            }
        }
    }
    getLeaves(tree); 


    // create lines
    List<Vector3> cellVerts = new List<Vector3>();
    var cellLines = new Line3D(mode: MeshVertexMode.Segments);
    {
        foreach(var l in leaves) {
            async void destr() {
                await Animate.Color(l.cell, l.cell.Color, Color.TRANSPARENT, 1.0f);
                world.Destroy(l.cell);
            }
            destr();
        }
        foreach(var seg in CellOuterEdges(tree.min, tree.size)) {
            cellVerts.Add(seg.s);
            cellVerts.Add(seg.e);
        }
        void traverse(VisNode node) {
            if (node.children[0] == null) {
                return;
            }
            foreach(var seg in CellInnerEdges(node.min, node.size)) {
                cellVerts.Add(seg.s);
                cellVerts.Add(seg.e);
            }
            foreach(var child in node.children) {
                if (child != null) traverse(child);
            }
        }
        traverse(tree);
        cellLines.Vertices = cellVerts.ToArray();
        cellLines.Color = Color.VIOLET;
        cellLines.Width = 3.0f;
        World.current.CreateInstantly(cellLines);
        await Time.WaitSeconds(0.25f);
    }

    List<(Vector3 p, Vector3 n, int dir)> intersections2 = new ();
    for(int i = 0; i < intersections.GetLength(1); i++) {
        for(int j = 0; j < intersections.GetLength(2); j++) {
            for(int k = 0; k < intersections.GetLength(3); k++) {
                for (int d = 0; d < 3; d++) {
                    var intr = intersections[d, i, j, k];
                    if (intr.HasValue) {
                        intersections2.Add((intr.Value.v, intr.Value.n, d));
                    }
                }
            }
        }
    }

    Line3D intrNormalLines = new(width: 5.0f, mode: MeshVertexMode.Segments);
    intrNormalLines.SortKey.Value = normalLines.SortKey - 1;
    intrNormalLines.Color = Color.ORANGE;
    world.CreateInstantly(intrNormalLines);
    Line3D highlightLines = new(width: 3.0f, mode: MeshVertexMode.Segments);
    highlightLines.SortKey.Value = normalLines.SortKey - 1;
    highlightLines.Color = Color.RED;
    highlightLines.Vertices = CubicalMS.CellOuterEdges(Vector3.ZERO, 1.0f).SelectMany(x => new Vector3[] { x.s, x.e }).ToArray();

    async Task showText(string txt) {
        var reasonText = new Text2D(txt);
        reasonText.Color = Color.WHITE;
        reasonText.Anchor = new Vector2(-0.25f, 0.2f);
        await Animate.CreateText(reasonText, TextCreationMode.Fade, 0.0f);
        await Time.WaitSeconds(0.5f);
        var tasks = reasonText.CurrentShapes.Select(async g => await Animate.Color(g.s, g.s.Color, Color.TRANSPARENT, 0.1f)).ToArray();
        await Task.WhenAll(tasks);
        world.Destroy(reasonText);
    }

    int caseCount = 0;

    _ = Animate.ChangeText(titleText, "CMS step 3: adaptive octree subdivision");
    var ruleText1 = new Text2D("Rule 1: Edge ambiguity (multiple intersections on cell edge) -> subdivide cell");
    ruleText1.Color = (Color.GREEN*0.85f).WithA(1.0f);
    ruleText1.Anchor = new Vector2(-0.45f, -0.3f);
    var ruleText2 = new Text2D("Rule 2: Dot product of any pair of intersecting normals is below a specified threshold (complex surface) -> subdivide cell");
    ruleText2.Color = (Color.GREEN*0.85f).WithA(1.0f);
    ruleText2.Anchor = new Vector2(-0.45f, -0.3f);
    ruleText2.Position = new Vector2(0.0f, -50.0f);
    await world.CreateFadeIn(ruleText1, 0.7f);
    await world.CreateFadeIn(ruleText2, 0.7f);

    var tasks = leaves.Select(l => Animate.Color(l.cell, l.cell.Color, Color.TRANSPARENT, 0.5f)).ToArray();
    await Task.WhenAll(tasks);
    async Task SubdiveIfNeeded(VisNode l, bool quick, int maxDepth) {
        quick = caseCount > 4;
        if (!quick) {
            var forward = Quaternion.AngleAxis(float.DegreesToRadians(45.0f), Vector3.UP) * Quaternion.AngleAxis(float.DegreesToRadians(45.0f), Vector3.RIGHT) * Vector3.FORWARD;
            var (pos, rot) = Animate.ObservingTransform(l.min, l.min+ new Vector3(l.size), forward, World.current.ActiveCamera as PerspectiveCamera, 1920.0f/1080.0f);
            await Animate.TransformToLimited(World.current.ActiveCamera, pos, rot, 0.5f, 5.0f);
        }
        if (!highlightLines.Created) {
            await world.CreateFadeIn(highlightLines, quick ? 0.5f : 0.5f);
            highlightLines.Position = l.min;
            highlightLines.Scale = new Vector3(l.size);
        } else {
            if (!quick) {
                await Task.WhenAll([Animate.Move(highlightLines, l.min, 0.5f),
                    Animate.Scale(highlightLines, new Vector3(l.size), 0.5f)]);
            }
        }

        if (leaves.Contains(l)) {
            //await Animate.Color(l.cell, l.cell.Color, lColor, 0.5f);
        }

        List<Sphere> spheres = new ();

        // TODO: find intersection on each cell edge
        List<HermiteIntersection> allIntr = new ();
        bool foundAny = false;
		bool foundX = false;
        List<(Vector3 s, Vector3 e)> normalLines = new ();
        for (int i = 0; i < 12; i++) {
            var intrs = findEdgeIntersections(hermiteData, l.min, l.size, i);
            foreach(var intr in intrs) {
                if (!quick) {
                    var sphere = new Sphere(0.05f);
                    sphere.Color = Color.YELLOW;
                    sphere.Position = intr.p;
                    world.CreateInstantly(sphere);
                    spheres.Add(sphere);
                }
                allIntr.Add(new HermiteIntersection() {
                    pos = intr.p,
                    normal = intr.n,
                });
                normalLines.Add((intr.p, intr.p + 0.5f * intr.n));
            }
            if (intrs.Length > 1) {
                foundAny = true;
            }
			if (intrs.Length > 0) {
				foundX = true;
			}
        }
        if (!quick) {
            intrNormalLines.Vertices = normalLines.SelectMany(x => new Vector3[] { x.s, x.e }).ToArray();
        }

        if (foundX) {
            caseCount++;
        }

        //await Time.WaitSeconds(quick ? 0.1f : 0.5f);
        bool complx = CMSCell.LikelyToContainComplexSurface(allIntr.ToArray());
        if ((complx && l.depth < maxDepth) || (foundAny && l.depth < maxDepth)) {
            var text = complx ? "Likely complex surface" : "Edge ambiguity";

            async Task doSub() {
                await l.SubdivideWithLines(cellVerts, quick);
                cellLines.Vertices = cellVerts.ToArray();
            }

            Task[] tasks = [quick ? Task.CompletedTask : showText(text), doSub()];
            await Task.WhenAll(tasks);
            World.current.Destroy(spheres.ToArray());
            intrNormalLines.Vertices = [];

            foreach(var child in l.children) {
                if (child != null) {
                    if (child.depth < 44) {
                        await SubdiveIfNeeded(child, quick, maxDepth);
                    }
                }
            }
        } else if (!quick) {
            string text = "";
            if (spheres.Count == 0) {
                text = "No intersections";
            } else if (l.depth >= 4) {
                text = "Depth limit reached";
            } else {
                text = "No edge ambiguity or complex surface";
            }
            await showText(text);
            World.current.Destroy(spheres.ToArray());
            intrNormalLines.Vertices = [];
        }
    }

    var originalPos = World.current.ActiveCamera.Position;
    var originalRot = World.current.ActiveCamera.Rotation;

    List<Task> subdiveTasks = new List<Task>();
    bool goingfast = false;
    foreach (var l in leaves) {
        if (caseCount < 3) {
            await SubdiveIfNeeded(l, false, hermiteDepth);
        } else {
            if (!goingfast) {
                await Time.WaitSeconds(1.0f);
            }
            goingfast = true;
            subdiveTasks.Add(SubdiveIfNeeded(l, true, hermiteDepth));
        }
    }

    _ = Animate.TransformTo(World.current.ActiveCamera, originalPos, originalRot, 1.0f);

    _ = world.DestroyFadeOut(ruleText1, 0.7f);
    _ = world.DestroyFadeOut(ruleText2, 0.7f);

    await Animate.Color(highlightLines, highlightLines.Color, Color.TRANSPARENT, 0.5f);
    world.Destroy(highlightLines);
    await Task.WhenAll(subdiveTasks);

    await Time.WaitSeconds(1.0f);

    tree.AssignIntersections(hermiteData);

    await Animate.Color(cellLines, cellLines.Color, cellLines.Color.WithA(0.1f), 0.5f);
    //world.Destroy(cellLines);

    _ = Animate.ChangeText(titleText, "CMS step 4: Evaluate cell faces");

    var testLine = new Line3D(mode: MeshVertexMode.Segments);
    testLine.Width = 3.0f;
    List<Vector3> testVerts = new List<Vector3>();
    List<Color> testColors = new List<Color>();
    void traverseCells(VisNode node) {
        for (int i = 0; i < 6; i++) {
            var face = node.faces[i];
            if (face.edges == null) continue;
            for (int j = 0; j < 4; j++) {
                var edge = face.edges[j];
                testVerts.Add(edge.s);
                testVerts.Add(edge.e);
            }
        }
        if (node.children[0] != null) {
            foreach(var child in node.children) {
                traverseCells(child);
            }
        }
    }

    Color[] faceColors = [Color.RED, Color.BLUE, Color.YELLOW, Color.GREEN, Color.CYAN, Color.MAGENTA];

    traverseCells(tree);
    testLine.Color = Color.CYAN;
    testLine.Vertices = testVerts.ToArray();
    //testLine.Colors = testColors.ToArray();
    testLine.Width = 2.0f;
    //world.CreateInstantly(testLine);

	//var segments = await tree.EvaluateFaces();
	var mesh = await tree.ExtractSurface(titleText);


    _ = Animate.ChangeText(titleText, "Final mesh");
    async Task fadeStuffOut() {
        Task[] tasks = [Animate.Color(cellLines, cellLines.Color, Color.TRANSPARENT, 0.5f), Animate.Color(normalLines, normalLines.Color, Color.TRANSPARENT, 0.5f),
        Animate.Color(testLine, testLine.Color, Color.TRANSPARENT, 0.5f)];
        await Task.WhenAll(tasks);
    world.Destroy(cellLines);
    }
    await fadeStuffOut();
    await Animate.OrbitAndLookAt(cam, Vector3.UP, Vector3.ZERO, 720.0f + 45.0f, 10.0f); 

    tree.SanityCheck();
    Debug.Log("Full sanity check passed!");

    await Time.WaitSeconds(1.0f);

    await Animate.Color(mesh, mesh.Color, Color.TRANSPARENT, 0.5f);
    world.Destroy(mesh);

    _ = Animate.ChangeText(titleText, "Glyph 64x64x64");

    await Time.WaitSeconds(0.5f);

    var jokeMesh = new Mesh();
    //jokeMesh.Culling = MeshCulling.DrawCw;
    jokeMesh.Color = Color.VIOLET;
    jokeMesh.Vertices = jmesh.vertices.Select(x => (Vector3)x).ToArray();
    jokeMesh.Scale = new Vector3(1.8f);
    jokeMesh.Indices = jmesh.indices.Select(x => (uint)x).ToArray();
    jokeMesh.Position = new Vector3(0.0f, -3.0f, 0.25f*19.2f);
    await World.current.CreateFadeIn(jokeMesh, 0.5f);

    var lorbitTask =  Animate.OrbitAndLookAt(cam, Vector3.UP, Vector3.ZERO, 1080.0f, 11.0f); 
    await Time.WaitSeconds(3.0f);
    
    await Animate.Color(jokeMesh, jokeMesh.Color, Color.TRANSPARENT, 0.5f);
    world.Destroy(jokeMesh);

    _ = Animate.ChangeText(titleText, "Glyph 128x128x128");

    hermiteData2 = MyCMS.HermiteData.FromSdf(myFunc3, myFunc3Normal, offset, gSize*0.125f, 8*gCount);
    fastTree = new MyCMS.CMSTree(hermiteData2);
    fastTree.Subdivide(2);
    jmesh = fastTree.ExtractSurface();

    jokeMesh.Color = Color.VIOLET;
    jokeMesh.Vertices = jmesh.vertices.Select(x => (Vector3)x).ToArray();
    jokeMesh.Indices = jmesh.indices.Select(x => (uint)x).ToArray();
    jokeMesh.Scale = new Vector3(1.8f);
    jokeMesh.Position = new Vector3(0.0f, -3.0f, 0.25f*19.2f);
    await World.current.CreateFadeIn(jokeMesh, 0.5f);

    await Time.WaitSeconds(2.0f);

    await Time.WaitSeconds(0.25f);
    await Animate.Color(jokeMesh, jokeMesh.Color, Color.TRANSPARENT, 0.5f);
    world.Destroy(jokeMesh);


    _ = Animate.ChangeText(titleText, "Glyph subtracted 32x32x32");
    hermiteData2 = MyCMS.HermiteData.FromSdf(myFuncLast2, myFuncLast2Normal, offset, gSize*0.5f, 2*gCount);
    fastTree = new MyCMS.CMSTree(hermiteData2);
    fastTree.Subdivide(2);
    jmesh = fastTree.ExtractSurface();
    jokeMesh.Color = Color.VIOLET;
    //jokeMesh.Position = new Vector3(0.0f, -3.0f, 0.25f*19.2f);
    jokeMesh.Vertices = jmesh.vertices.Select(x => (Vector3)x).ToArray();
    jokeMesh.Indices = jmesh.indices.Select(x => (uint)x).ToArray();
    await World.current.CreateFadeIn(jokeMesh, 0.5f);

    await Time.WaitSeconds(0.5f);
    await lorbitTask;

    await Animate.Color(jokeMesh, jokeMesh.Color, Color.TRANSPARENT, 0.5f);
    world.Destroy(jokeMesh);

    static float TestEval(System.Numerics.Vector3 pos) {
        //return sdTorus(pos);
        //var sphere = sdSphere((Vector3)pos - new Vector3(3.0f, 2.9f, 3.0f), 0.5f);
        //var sphere2 = sdSphere((Vector3)pos - new Vector3(4.4f, 2.8f, 3.0f), 0.5f);
        var rot = M3x3.Rotate(Quaternion.AngleAxis(-0.25f* MathF.PI, Vector3.RIGHT)* Quaternion.AngleAxis(0.13f* MathF.PI, Vector3.FORWARD)* Quaternion.AngleAxis(-0.3f* MathF.PI, Vector3.UP)) * M3x3.Scale(0.33f, 0.33f, 0.33f);
        var pyr = sdPyramid(rot*(Vector3)pos - 0.33f*new Vector3(2.9f, 4.5f, 2.8f), 0.3f, 0.3f, 1.25f);
        //var cube = sdBox(rot*(Vector3)pos - 0.33f*new Vector3(3.0f, 1.9f, 2.8f), new Vector3(0.6f));
        //return opSmoothUnion(sphere, sphere2, k: 0.5f);
        return pyr;
    }

    Func<System.Numerics.Vector3, System.Numerics.Vector3> testNormal = 
        (System.Numerics.Vector3 pos) => {
            return CalcNormal(x => TestEval(x), pos, eps: 0.0033f);
    };

    MyCMS.HermiteData testHData = MyCMS.HermiteData.FromSdf(TestEval, testNormal, new System.Numerics.Vector3(0.0f), 1.0f, 8);
    for (int j = 0; j < 8; j++) {
        for (int i = 0; i < 8; i++) {
            System.Console.Write($"{testHData.isInside[i, j, 3]} ");
        }
        System.Console.WriteLine();
    }
    fastTree = new MyCMS.CMSTree(testHData);
    fastTree.Subdivide(3);
    jmesh = fastTree.ExtractSurface();
    jokeMesh.Color = Color.VIOLET;
    jokeMesh.Vertices = jmesh.vertices.Select(x => (Vector3)x).ToArray();
    jokeMesh.Indices = jmesh.indices.Select(x => (uint)x).ToArray();
    jokeMesh.Scale = new Vector3(1.0f);
    jokeMesh.Position = Vector3.ZERO;
    System.Console.WriteLine($"Generated {jmesh.vertices.Length} vertices for test mesh");
    System.Console.WriteLine($"Generated {jmesh.indices.Length/3} triangles for test mesh");
    World.current.CreateInstantly(jokeMesh);

    var normLines = new Line3D(mode: MeshVertexMode.Segments);
    normLines.Color = Color.GREEN;
    List<Vector3> nlines = new();
    List<Color> ncolors = new();
    for (int d = 0; d < 3; d++)
    for (int i = 0; i < 8; i++)
    for (int j = 0; j < 8; j++)
    for (int k = 0; k < 8; k++) {
        var intr = testHData.intersections[d, i, j, k];
        if (intr.HasValue) {
            nlines.Add(intr.Value.p);
            nlines.Add(intr.Value.p + intr.Value.n * 0.5f);
            ncolors.Add(d switch {
                0 => Color.RED,
                1 => Color.GREEN,
                2 => Color.BLUE,
                _ => Color.WHITE
            });
            ncolors.Add(ncolors.Last());
        }
    }
    normLines.Vertices = nlines.ToArray();
    normLines.Colors = ncolors.ToArray();
    World.current.CreateInstantly(normLines);

    var egrid = make3DGrid(new Vector3(4.0f, 0.0f, 0.0f), 1.0f, 4);
    var egridLines = new Line3D();
    egridLines.Color = (Color.WHITE*0.5f).WithA(0.2f);
    egridLines.Width = 3.0f;
    egridLines.Vertices = egrid.ToArray();
    var egridColor = egridLines.Color;
    world.CreateInstantly(egridLines);

    await Time.WaitSeconds(2.0f);
}

async Task AnimateAmbiguous3D() {
    var allEntities = world.BeginCapture();

    var cam = world.ActiveCamera;
    cam.Position = new Vector3(0.0f, 0.0f, -5.0f);
    cam.Rotation = Quaternion.LookRotation(Vector3.FORWARD, Vector3.UP);

    var orbitTask = Animate.OrbitAndLookAt(cam, Vector3.UP, Vector3.ZERO, 720.0f, 20.0f);

    var offset1 = new Vector3(0.5f, -0.5f, -0.5f); 
    var offset2 = new Vector3(-1.5f, -0.5f, -0.5f); 
    // create 8 cube corners
    for (int i = 0; i < 8; i++) {
        var pos = CMS.cornerOffsets[i];
        var s = new Sphere(0.05f);
        s.Color = (i == 0 || i == 6) ? new Color(0.5f, 0.5f, 0.5f, 1.0f) :Color.BLACK;
        s.Position = pos + offset1;
        var s2 = world.Clone(s);
        s2.Position = pos + offset2;
        world.CreateInstantly(s);
        world.CreateInstantly(s2);
    }

    var exampleMesh1a = new Mesh();
    exampleMesh1a.Color = new Color(1.0f, 0.05f, 0.05f, 1.0f);
    exampleMesh1a.Outline = Color.BLACK;
    exampleMesh1a.Position = offset1;
    exampleMesh1a.Vertices = new Vector3[] {
        (0.56f, -0.58f, -0.35f),
        (0.1f, 0.5f, 0.2f),
        (0.2f, -0.5f, 0.6f),
        (0.2f, 0.2f, 0.3f),
        (-0.5f, 0.5f, 0.1f),
        (-0.6f, -0.5f, 0.55f), 
        (0.0f, -0.5f, -0.58f),
        (-0.6f, -0.5f, -0.53f), 
    };
    exampleMesh1a.Indices = new uint[] { 
        3, 0, 1,
        1, 3, 2,
        2, 3, 0,
        1, 4, 5,
        1, 5, 2,
        1, 4, 7,
        7, 1, 6,
        1, 6, 0,
        6, 2, 0,
        6, 7, 2,
        7, 5, 2,
        7, 5, 4,
    };
    world.CreateInstantly(exampleMesh1a);
    var exampleMesh1b = world.Clone(exampleMesh1a);
    exampleMesh1b.Position = offset1 + new Vector3(1.0f, 1.0f, 1.0f);
    var rot = Quaternion.AngleAxis(MathF.PI, Vector3.FORWARD);
    rot = rot * Quaternion.AngleAxis(MathF.PI, Vector3.UP);
    exampleMesh1b.Rotation = rot;
    world.CreateInstantly(exampleMesh1b);

    var exampleMesh2 = new Mesh();
    exampleMesh2.Color = new Color(1.0f, 0.05f, 0.05f, 1.0f);
    exampleMesh2.Outline = Color.BLACK;
    exampleMesh2.Position = offset2;
    exampleMesh2.Vertices = new Vector3[] {
        (0.55f, 0.1f, 0.15f),
        (0.1f, 0.4f, 0.1f),
        (0.034f, 0.07f, 0.32f),

        (0.60f, 0.7f, 1.01f),
        (0.95f, 0.4f, 0.70f),
        (0.8f, 0.85f, 0.6f),

        (0.5f, -0.05f, 0.0f) - new Vector3(0.5f) - new Vector3(-0.35f, -0.25f, -0.15f),
        (0.5f, 1.0f, 1.0f) + new Vector3(0.5f) + new Vector3(0.35f, -0.05f, -0.75f),

        (0.5f, -0.05f, 0.0f) - new Vector3(0.5f) - new Vector3(0.15f, -1.05f, 0.05f),
        (0.5f, -0.05f, 0.0f) - new Vector3(0.5f) - new Vector3(0.35f, -0.5f, -0.85f),

        (0.5f, 1.0f, 1.0f) + new Vector3(0.5f) + new Vector3(-0.25f, -0.45f, 0.25f),
        (0.5f, 1.0f, 1.0f) + new Vector3(0.5f) + new Vector3(0.35f, -0.85f, -0.25f),
    };
    exampleMesh2.Indices = new uint[] { 
        0, 5, 1,
        0, 5, 4,
        1, 3, 5,
        1, 3, 2,
        0, 2, 4,
        2, 4, 3,

        0, 6, 1,
        1, 6, 8,
        2, 1, 8,
        2, 8, 9,
        2, 6, 0,
        9, 6, 2,

        3, 7, 5,
        3, 10, 7,
        5, 7, 4,
        4, 7, 11,
        3, 4, 11,
        3, 11, 10,

        6, 9, 8,
        10, 11, 7,
    };
    world.CreateInstantly(exampleMesh2);

    var cubeLines = new Line3D();
    cubeLines.Color = Color.BLACK;
    cubeLines.Width = 4.0f;
    Vector3[] vs = new Vector3[24];
    for (int i = 0; i < 12; i++) {
        var (c1, c2) = CMS.cubeEdgeCorners[i];
        vs[2*i] = CMS.cornerOffsets[c1];
        vs[2*i+1] = CMS.cornerOffsets[c2];
    }
    cubeLines.Vertices = vs;
    cubeLines.Position = offset1;
    cubeLines.Color = Color.WHITE;
    var cubeLines2 = world.Clone(cubeLines);
    cubeLines2.Position = offset2;
    world.CreateInstantly(cubeLines);
    world.CreateInstantly(cubeLines2);

    (Vector3 v, Vector3 n)[] getIntersections(Mesh imesh, Vector3 o) {
        List<(Vector3 v, Vector3 n)> intersections = new();
        for (int i = 0; i < imesh.Indices.Length; i+=3) {
            Vector3 p1, p2, p3;
            p1 = imesh.Rotation * imesh.Vertices[imesh.Indices[i]] + imesh.Position;
            p2 = imesh.Rotation * imesh.Vertices[imesh.Indices[i+1]] + imesh.Position;
            p3 = imesh.Rotation * imesh.Vertices[imesh.Indices[i+2]] + imesh.Position;

            for (int e = 0; e < 12; e++) {
                var start = CMS.cornerOffsets[CMS.cubeEdgeCorners[e].Item1] + imesh.Position + o;
                var end = CMS.cornerOffsets[CMS.cubeEdgeCorners[e].Item2] + imesh.Position + o;

                var intersection = AMath.IntersectSegmentTriangle(start, end, p1, p2, p3);

                if (intersection != null) {
                    var normal = Vector3.Cross(p2 - p1, p3 - p1).Normalized;
                    intersections.Add((intersection.Value, normal));
                }
            }
        }
        return intersections.ToArray();
    }

    var intersections11 = getIntersections(exampleMesh1a, Vector3.ZERO);
    var intersections12 = getIntersections(exampleMesh1b, -Vector3.ONE);
    // [ 1, 2, 3] are the first triangle
    var intersections2 = getIntersections(exampleMesh2, Vector3.ZERO);

    var allIntersections = new List<(Vector3 v, Vector3 n)>();
    allIntersections.AddRange(intersections11);
    allIntersections.AddRange(intersections12);
    allIntersections.AddRange(intersections2);
    var normalLines = new Line3D();
    normalLines.Color = Color.GREEN;
    normalLines.Width = 5.0f;
    normalLines.Vertices = allIntersections.SelectMany(i => new Vector3[] { i.v, i.v + 0.3f*i.n }).ToArray();
    world.CreateInstantly(normalLines);

    var intersections21 = intersections2.Where(x => (x.v - exampleMesh2.Position).z < 0.5f).ToArray();
    var intersections22 = intersections2.Where(x => (x.v - exampleMesh2.Position).z > 0.5f).ToArray();

    Vector3 calcPoint((Vector3 v, Vector3 n)[] intersections) {
        var matrix = Matrix<float>.Build.Dense(intersections.Length, 3);
        var b = Vector<float>.Build.Dense(intersections.Length);
        for (int i = 0; i < intersections.Length; i++) {
            var n = intersections[i].n;
            matrix.SetRow(i, (float[]) [ n.x, n.y, n.z ]);
            b[i] = Vector3.Dot(n, intersections[i].v);
        }
        var pinv = matrix.PseudoInverse();
        var res = pinv * b;
        return new Vector3(res[0], res[1], res[2]);
    }
    var sharp11 = calcPoint(intersections11);
    var sharp12 = calcPoint(intersections12);
    var sharp21 = calcPoint(intersections21);
    var sharp22 = calcPoint(intersections22);

    Debug.Log($"sharp11: {sharp11}");
    Debug.Log($"sharp12: {sharp12}");
    Debug.Log($"sharp21: {sharp21}");
    Debug.Log($"sharp22: {sharp22}");

    await Time.WaitSeconds(6.0f);

    // fade out meshes  
    float innerAlpha = 0.1f;
    float outerAlpha = 0.3f;
    await Animate.InterpF(x => {
        exampleMesh1a.Color = Color.Lerp(exampleMesh1a.Color, exampleMesh1a.Color.WithA(innerAlpha), x);
        exampleMesh1a.Outline = Color.Lerp(exampleMesh1a.Outline, exampleMesh1a.Outline.WithA(outerAlpha), x);
        exampleMesh1b.Color = Color.Lerp(exampleMesh1b.Color, exampleMesh1b.Color.WithA(innerAlpha), x);
        exampleMesh1b.Outline = Color.Lerp(exampleMesh1b.Outline, exampleMesh1b.Outline.WithA(outerAlpha), x);
        exampleMesh2.Color = Color.Lerp(exampleMesh2.Color, exampleMesh2.Color.WithA(innerAlpha), x);
        exampleMesh2.Outline = Color.Lerp(exampleMesh2.Outline, exampleMesh2.Outline.WithA(outerAlpha), x);
    }, 0.0f, 1.0f, 0.5f);

    // connect triangle vertices
    var triangleLines = new Line3D();
    triangleLines.Color = Color.YELLOW;
    triangleLines.Width = 6.0f;
    triangleLines.Vertices = new Vector3[] {
        intersections2[0].v,
        intersections2[1].v,
        intersections2[1].v,
        intersections2[2].v,
        intersections2[2].v,
        intersections2[0].v,

        intersections2[3].v,
        intersections2[4].v,
        intersections2[4].v,
        intersections2[5].v,
        intersections2[5].v,
        intersections2[3].v,

        intersections11[0].v,
        intersections11[1].v,
        intersections11[1].v,
        intersections11[2].v,
        intersections11[2].v,
        intersections11[0].v,

        intersections12[0].v,
        intersections12[1].v,
        intersections12[1].v,
        intersections12[2].v,
        intersections12[2].v,
        intersections12[0].v,
    };
    await world.CreateFadeIn(triangleLines, 0.5f);

    await Time.WaitSeconds(1.0f);

    Vector3[] sharpFeatures = [ sharp11, sharp12, sharp21, sharp22 ];
    Task tttt = null;
    foreach (var sf in sharpFeatures) {
        var s = new Sphere(0.02f);
        s.Color = 1.5f*Color.RED;
        s.Position = sf;
        async Task fade() {
            await world.CreateFadeIn(s, 0.5f);
            await Animate.InterpF(x => {
                s.Color = Color.Lerp(1.5f*Color.RED, Color.RED, x);
            }, 0.0f, 1.0f, 0.5f);
        }
        tttt = fade(); 
    }
    await tttt;

    await Time.WaitSeconds(1.0f);

    var fanColor = new Color("#baac7d");

    var recFan11 = new Mesh();
    recFan11.Color = fanColor;
    recFan11.Outline = Color.BLACK;
    recFan11.Vertices = new Vector3[] {
        intersections11[0].v,
        intersections11[1].v,
        intersections11[2].v,
        sharp11,
    };
    recFan11.Indices = new uint[] { 0, 1, 3, 1, 2, 3, 2, 0, 3 };

    var recFan12 = new Mesh();
    recFan12.Color = fanColor;
    recFan12.Outline = Color.BLACK;
    recFan12.Vertices = new Vector3[] {
        intersections12[0].v,
        intersections12[1].v,
        intersections12[2].v,
        sharp12,
    };
    recFan12.Indices = new uint[] { 0, 1, 3, 1, 2, 3, 2, 0, 3 };

    var recFan21 = new Mesh();
    recFan21.Color = fanColor;
    recFan21.Outline = Color.BLACK;
    recFan21.Vertices = new Vector3[] {
        intersections21[0].v,
        intersections21[1].v,
        intersections21[2].v,
        sharp21,
    };
    recFan21.Indices = new uint[] { 0, 1, 3, 1, 2, 3, 2, 0, 3 };

    var recFan22 = new Mesh();
    recFan22.Color = fanColor;
    recFan22.Outline = Color.BLACK;
    recFan22.Vertices = new Vector3[] {
        intersections22[0].v,
        intersections22[1].v,
        intersections22[2].v,
        sharp22,
    };
    recFan22.Indices = new uint[] { 0, 1, 3, 1, 2, 3, 2, 0, 3 };

    _ = world.CreateFadeIn(recFan11, 0.5f);
    _ = world.CreateFadeIn(recFan12, 0.5f);
    _ = world.CreateFadeIn(recFan21, 0.5f);
    var ttt = world.CreateFadeIn(recFan22, 0.5f);
    await ttt;

    await Time.WaitSeconds(1.0f);

    // fade out recFanc21 and recFan22 
    await Animate.InterpF(x => {
        recFan21.Color = Color.Lerp(recFan21.Color, recFan21.Color.WithA(0.0f), x);
        recFan21.Outline = Color.Lerp(recFan21.Outline, recFan21.Outline.WithA(0.0f), x);
        recFan22.Color = Color.Lerp(recFan22.Color, recFan22.Color.WithA(0.0f), x);
        recFan22.Outline = Color.Lerp(recFan22.Outline, recFan22.Outline.WithA(0.0f), x);
    }, 0.0f, 1.0f, 0.5f);
    world.Destroy(recFan21);
    world.Destroy(recFan22);

    var recFan2Alt = new Mesh();
    recFan2Alt.Color = fanColor;
    recFan2Alt.Outline = Color.BLACK;
    recFan2Alt.Vertices = new Vector3[] {
        intersections2[0].v,
        intersections2[1].v,
        intersections2[2].v,
        intersections2[3].v,
        intersections2[4].v,
        intersections2[5].v,
    };
    recFan2Alt.Indices = new uint[] { 0, 1, 4, 1, 2, 3, 4, 3, 1, 2, 0, 5, 5, 3, 2, 0, 4, 5};
    await world.CreateFadeIn(recFan2Alt, 0.5f);

    await orbitTask;

    world.EndCapture();

    foreach (dynamic s in allEntities) {
        _ = Animate.InterpF(x => {
            try {
                s.Color = Color.Lerp(s.Color, Color.TRANSPARENT, x);
                s.Outline = Color.Lerp(s.Outline, Color.TRANSPARENT, x);
            } catch(Exception) {}
        }, 0.0f, 1.0f, 0.5f);
    }
    await Time.WaitSeconds(0.5f);
    foreach (var s in allEntities) {
        if (! s.ManagedLifetime) {
            world.Destroy(s);
        }
    }

    _ = Animate.ChangeText(titleText, "");
}

async Task AnimateAmbiguous2D() {
    titleText.Anchor = new Vector2(-0.3f, titleText.Anchor.y);
    _ = Animate.ChangeText(titleText, "2D face ambiguities");

    var allEntities = world.BeginCapture();

    var gridPath1 = new PathBuilder();
    gridPath1.Grid(75.0f, new Vector2(-525.0f, -225.0f), new Vector2(-75.0f, 225.0f));
    var gridPath2 = new PathBuilder();
    gridPath2.Grid(75.0f, new Vector2(75.0f, -225.0f), new Vector2(525.0f, 225.0f));

    var gridShape1 = new Shape(gridPath1);
    gridShape1.ContourColor = Color.WHITE;
    world.CreateInstantly(gridShape1);
    var gridShape2 = new Shape(gridPath2);
    gridShape2.ContourColor = Color.WHITE;
    world.CreateInstantly(gridShape2);

    var examplePath = new PathBuilder();
    examplePath.MoveTo(new Vector2(-225.0f, 90.0f));
    examplePath.LineTo(new Vector2(-60.0f, -75.0f));
    examplePath.LineTo(new Vector2(-30.0f, 22.5f));
    examplePath.LineTo(new Vector2(90.0f, 22.5f));
    examplePath.LineTo(new Vector2(-30.0f, 112.5f));
    examplePath.LineTo(new Vector2(-30.0f, 52.5f));
    examplePath.Close();
    var exampleShape = new Shape(examplePath);
    exampleShape.Mode = ShapeMode.FilledContour;
    exampleShape.ContourColor = Color.BLACK;
    exampleShape.Color = new Color(1.0f, 0.1f, 0.1f, 0.5f);
    exampleShape.Position = new Vector3(-225.0f, 0.0f, 0.0f);
    world.CreateInstantly(exampleShape);

    var examplePath2a = new PathBuilder();
    examplePath2a.MoveTo(new Vector2(-225.0f, 90.0f));
    examplePath2a.LineTo(new Vector2(-60.0f, -75.0f));
    examplePath2a.LineTo(new Vector2(-45.0f, 22.5f));
    examplePath2a.Close();

    var examplePath2b = new PathBuilder();
    examplePath2b.MoveTo(new Vector2(90.0f, 67.5f));
    examplePath2b.LineTo(new Vector2(-30.0f, 112.5f));
    examplePath2b.LineTo(new Vector2(-30.0f, 37.5f));
    examplePath2b.Close();

    var exampleShape2a = new Shape(examplePath2a);
    exampleShape2a.Mode = ShapeMode.FilledContour;
    exampleShape2a.ContourColor = Color.BLACK;
    exampleShape2a.Color = new Color(1.0f, 0.1f, 0.1f, 0.5f);
    exampleShape2a.Position = new Vector3(375.0f, 0.0f, 0.0f);
    world.CreateInstantly(exampleShape2a);
    var exampleShape2b = new Shape(examplePath2b);
    exampleShape2b.Mode = ShapeMode.FilledContour;
    exampleShape2b.ContourColor = Color.BLACK;
    exampleShape2b.Color = new Color(1.0f, 0.1f, 0.1f, 0.5f);
    exampleShape2b.Path = examplePath2b;
    exampleShape2b.Position = new Vector3(375.0f, 0.0f, 0.0f);
    world.CreateInstantly(exampleShape2b);

    // yellow highlight

    var highlightPath1 = new PathBuilder();
    highlightPath1.Rectangle(new Vector2(-300.0f, 0.0f), new Vector2(-225.0f, 75.0f));
    var highlightPath2 = new PathBuilder();
    highlightPath2.Rectangle(new Vector2(300.0f, 0.0f), new Vector2(375.0f, 75.0f));
    var highlightShape1 = new Shape(highlightPath1);  
    highlightShape1.Mode = ShapeMode.Contour;
    highlightShape1.ContourColor = 1.2f*Color.VIOLET;
    highlightShape1.SortKey.Value = 1;
    highlightShape1.ContourSize = 3.0f;
    world.CreateInstantly(highlightShape1);
    var highlightShape2 = new Shape(highlightPath2);  
    highlightShape2.Mode = ShapeMode.Contour;
    highlightShape2.ContourColor = 1.2f*Color.VIOLET;
    highlightShape2.SortKey.Value = 1;
    highlightShape2.ContourSize = 3.0f;
    world.CreateInstantly(highlightShape2);


    Vector2[] quadCorners1 = new Vector2[] {
        new Vector2(-300.0f, 0.0f),
        new Vector2(-225.0f, 0.0f),
        new Vector2(-225.0f, 75.0f),
        new Vector2(-300.0f, 75.0f),
    };

    Vector2[] quadCorners2 = new Vector2[] {
        new Vector2(300.0f, 0.0f),
        new Vector2(375.0f, 0.0f),
        new Vector2(375.0f, 75.0f),
        new Vector2(300.0f, 75.0f),
    };

    await Time.WaitSeconds(2.0f);

    bool[] cornerValues = new bool[] { true, false, true, false };
    Circle[] cornerCircles1 = new Circle[4];
    Circle[] cornerCircles2 = new Circle[4];
    for (int i = 0; i < 4; i++) {
        var c1 = new Circle(7.5f);
        c1.Color = cornerValues[i] ? Color.WHITE : Color.BLACK;
        c1.Position = quadCorners1[i];
        c1.SortKey.Value = 2;
        _ = world.CreateFadeIn(c1, 0.5f);
        cornerCircles1[i] = c1;
        var c2 = new Circle(7.5f);
        c2.Color = cornerValues[i] ? Color.WHITE : Color.BLACK;
        c2.Position = quadCorners2[i];
        c2.SortKey.Value = 2;
        await world.CreateFadeIn(c2, 0.5f);
        cornerCircles2[i] = c2;
    }

    await Time.WaitSeconds(1.0f);

    var caseText1 = new Text2D("1010");
    caseText1.Position = new Vector3(-375.0f, -300.0f, 0.0f);
    caseText1.Size = 64.0f;
    var shapes = caseText1.CurrentShapes.Select(x => x.s).ToArray();
    shapes[0].Color = Color.WHITE;
    shapes[1].Color = Color.BLACK;
    shapes[2].Color = Color.WHITE;
    shapes[3].Color = Color.BLACK;
    var caseText2 = world.Clone(caseText1);
    caseText2.Position = new Vector3(225.0f, -300.0f, 0.0f);
    var shapes2 = caseText2.CurrentShapes.Select(x => x.s).ToArray();
    shapes2[0].Color = Color.WHITE;
    shapes2[1].Color = Color.BLACK;
    shapes2[2].Color = Color.WHITE;
    shapes2[3].Color = Color.BLACK;

    Shape[] mcs1 = new Shape[4];
    Shape[] mcs2 = new Shape[4];
    Task<Shape>[] mcs2Tasks = new Task<Shape>[4];
    Task<Shape>[] mcs1Tasks = new Task<Shape>[4];
    async Task createChar() {
        Task task = null;
        for (int i = 0; i < 4; i++) {
            mcs1[i] = world.CreateClone(cornerCircles1[i]);
            mcs2[i] = world.CreateClone(cornerCircles2[i]);
            task =  Animate.Move(mcs1[i], caseText1.Position+shapes[i].Position);
            task =  Animate.Move(mcs2[i], caseText2.Position+shapes[i].Position);
        }
        await task;
        for (int i = 0; i < 4; i++) {
            task = mcs1Tasks[i] = Animate.CreateMorph(mcs1[i], shapes[i], 1.0f);
            mcs2Tasks[i] = Animate.CreateMorph(mcs2[i], shapes2[i], 1.0f);
        }
        await task;
    }
    await createChar();

    async Task showCase(int[] edges, Vector2 pos, string label, bool invert = false) {
        PathBuilder pb = new PathBuilder();
        pb.Rectangle(new Vector2(0.0f), new Vector2(75.0f));
        var shape = new Shape(pb);
        shape.Mode = ShapeMode.Contour;
        shape.ContourColor = Color.WHITE;
        shape.ContourSize = 3.0f;
        shape.Position = new Vector3(pos, 0.0f);
        shape.SortKey.Value = 1;
        _ = world.CreateFadeIn(shape, 0.5f);
        for (int i = 0; i < edges.Length; i+=2) {
            var (c11, c12) = CMS.quadEdgeCorners[edges[i]];
            var (c21, c22) = CMS.quadEdgeCorners[edges[i+1]];
            var pb2 = new PathBuilder();
            var repCheck = new int[] { c11, c12, c21, c22 };
            // find repeated corner
            if (!invert) {
                pb2.MoveTo(37.5*(CMS.quadCorners[c11] + CMS.quadCorners[c12]));
                pb2.LineTo(37.5*(CMS.quadCorners[c21] + CMS.quadCorners[c22]));
                int rep = -1;
                for (int j = 0; j < 4; j++) {
                    if (repCheck.Count(x => x == repCheck[j]) > 1) {
                        rep = repCheck[j];
                        break;
                    }
                }
                pb2.LineTo(75*CMS.quadCorners[rep]);
            } else {
                pb2.MoveTo(75f*new Vector2(0.0f));
                pb2.LineTo(75f*new Vector2(0.5f, 0.0f));
                pb2.LineTo(75f*new Vector2(1.0f, 0.5f));
                pb2.LineTo(75f*new Vector2(1.0f, 1.0f));
                pb2.LineTo(75f*new Vector2(0.5f, 1.0f));
                pb2.LineTo(75f*new Vector2(0.0f, 0.5f));
            }
            var shape2 = new Shape(pb2);
            shape2.Mode = ShapeMode.FilledContour;
            shape2.ContourColor = Color.WHITE;
            shape2.ContourSize = 3.0f;
            shape2.Position = new Vector3(pos, 0.0f);
            _ = world.CreateFadeIn(shape2, 0.5f);
        }
        var labelShape = new Text2D(label);
        labelShape.Size = 24.0f;
        labelShape.Color = Color.WHITE;
        labelShape.Position = pos + new Vector2(-15.0f, 90.0f);
        _ = world.CreateFadeIn(labelShape, 0.5f);
        await Time.WaitSeconds(0.5f);
    }

    await showCase(new int[] { 1, 0, 3, 2}, new Vector2(-75.0f, 262.5f), "10a (1010)");
    await showCase(new int[] { 3, 0, 1, 2}, new Vector2(75.0f, 262.5f), "10b (1010)", invert: true);


    await Time.WaitSeconds(1.0f);

    async Task moveGrid2() {
        float moveAmount = 1500.0f;
        var task = Animate.Offset(gridShape2, new Vector3(moveAmount, 0.0f, 0.0f));
        foreach(var shape in shapes2) {
            task = Animate.Offset(shape, new Vector3(moveAmount, 0.0f, 0.0f));
        }
        _ = Animate.Offset(exampleShape2a, new Vector3(moveAmount, 0.0f, 0.0f));
        _ = Animate.Offset(exampleShape2b, new Vector3(moveAmount, 0.0f, 0.0f));
        foreach(var cc in cornerCircles2) {
            _ = Animate.Offset(cc, new Vector3(moveAmount, 0.0f, 0.0f));
        }
        _ = Animate.Offset(highlightShape2, new Vector3(moveAmount, 0.0f, 0.0f));
        for (int i = 0; i < 4; i ++) {
            _ = Animate.Offset(mcs2Tasks[i].Result, new Vector3(moveAmount, 0.0f, 0.0f));
        }

        moveAmount = 337.5f;
        task = Animate.Offset(gridShape1, new Vector3(moveAmount, 0.0f, 0.0f));
        foreach(var shape in shapes) {
            task = Animate.Offset(shape, new Vector3(moveAmount, 0.0f, 0.0f));
        }
        _ = Animate.Offset(exampleShape, new Vector3(moveAmount, 0.0f, 0.0f));
        foreach(var cc in cornerCircles1) {
            _ = Animate.Offset(cc, new Vector3(moveAmount, 0.0f, 0.0f));
        }
        _ = Animate.Offset(highlightShape1, new Vector3(moveAmount, 0.0f, 0.0f));
        for (int i = 0; i < 4; i ++) {
            _ = Animate.Offset(mcs1Tasks[i].Result, new Vector3(moveAmount, 0.0f, 0.0f));
        }


        await task;
    }
    await moveGrid2();


    var gridBase = new Vector2(37.5f, 0.0f);

    (Vector2, Vector2, bool)[] intersections1 = new (Vector2, Vector2, bool)[] {
        (new Vector2(37.5f, 0.0f), new Vector2(1.0f, -0.3f).Normalized, true),
        (new Vector2(75.0f, 24.75f), new Vector2(0.0f, -1.0f).Normalized, false),
        (new Vector2(0.0f, 60.0f), new Vector2(0.15f, 1.0f).Normalized, false),
        (new Vector2(45.0f, 75.0f), new Vector2(-1.0f, 0.0f).Normalized, true),
    };

    var caseLinePath = new PathBuilder();
    caseLinePath.MoveTo(new Vector2(-15.0f, 0.0f));
    caseLinePath.LineTo(new Vector2(90.0f, 0.0f));
    var caseLine = new Shape(caseLinePath);
    caseLine.Mode = ShapeMode.Contour;
    caseLine.ContourColor = 1.3f*Color.YELLOW;
    caseLine.ContourSize = 3.0f;
    caseLine.Position = new Vector2(-75.0f, 247.5f);
    caseLine.SortKey.Value = 1;
    world.CreateInstantly(caseLine);
    await Animate.InterpF(x => {
        caseLine.ContourColor = Color.Lerp(Color.TRANSPARENT, 1.3f*Color.YELLOW, x);
    }, 0.0f, 1.0f, 0.5f);

    List<Shape> intersectionShapes = new List<Shape>();
    foreach (var intr in intersections1) {
        var c = new Circle(6.0f);
        c.Color = Color.YELLOW;
        c.Position = intr.Item1 + gridBase;
        c.SortKey.Value = 2;
        var pb = new PathBuilder();
        pb.MoveTo(intr.Item1);
        pb.LineTo(intr.Item1 + 22.5f*intr.Item2);
        var l = new Shape(pb);
        l.Mode = ShapeMode.Contour;
        l.ContourColor = Color.GREEN;
        l.Position = gridBase;
        l.SortKey.Value = 3;
        l.ContourSize = 4.0f;
        await world.CreateFadeIn(c, 0.5f);
        await world.CreateFadeIn(l, 0.5f);
        intersectionShapes.AddRange([c, l]);
    }

    await Time.WaitSeconds(1.0f);

    await Animate.InterpF(x => {
        exampleShape.Color = Color.Lerp(exampleShape.Color, Color.TRANSPARENT, x);
        exampleShape.ContourColor = Color.Lerp(exampleShape.ContourColor, Color.TRANSPARENT, x);
    }, 0.0f, 1.0f, 1.0f);

    await Time.WaitSeconds(1.0f);

    (int, int, Vector2)[] pairings1 = new (int, int, Vector2)[] {
        (0, 2, new Vector2(0.71f, 0.69f)),
        (1, 3, new Vector2(0.60f, 0.32f)),
    };

    (int, int, Vector2)[] pairings2 = new (int, int, Vector2)[] {
        (0, 1, new Vector2(0.60f, 0.32f)),
        (2, 3, new Vector2(0.61f, 0.71f)),
    };

    var col = new Color(0.1f, 0.4f, 1.0f, 1.0f);

    _ = Animate.Color(intersectionShapes[0], Color.YELLOW, col, 1.0f);
    await Animate.Color(intersectionShapes[4], Color.GREEN, col, 1.0f);

    async Task<List<Shape>> tryExample((int, int, Vector2)[] pairings) {
        List<Shape> garbage = new List<Shape>();
        foreach (var (p1, p2, v) in pairings) {
            var els = new int[] { p1, p2 };
            Task task = Task.CompletedTask;
            List<Shape> nws = new List<Shape>();
            foreach (var i in els) {
                var tpb = new PathBuilder();
                tpb.MoveTo(intersections1[i].Item1);
                float len = 75.0f;
                tpb.LineTo(intersections1[i].Item1 
                    + (!intersections1[i].Item3 ? len*intersections1[i].Item2.PerpCw : len*intersections1[i].Item2.PerpCcw));
                var tl = new Shape(tpb);
                tl.Mode = ShapeMode.Contour;
                tl.ContourColor = Color.MAGENTA;
                tl.Position = gridBase;
                tl.SortKey.Value = 4;
                tl.ContourSize = 2.0f;
                task = world.CreateFadeIn(tl, 0.5f);
                nws.Add(tl);
            }
            var intr = new Circle(6.0f);
            intr.Color = Color.ORANGE;
            intr.Position = gridBase + v*75.0f;
            intr.SortKey.Value = 2;
            task = world.CreateFadeIn(intr, 0.5f);
            garbage.Add(intr);
            await task;
            await Time.WaitSeconds(0.5f);
            foreach (var s in nws) {
                world.Destroy(s);
            }
            var pb = new PathBuilder();
            pb.MoveTo(intersections1[p1].Item1);
            pb.LineTo(v*75.0f);
            pb.LineTo(intersections1[p2].Item1);
            var l = new Shape(pb);
            l.Mode = ShapeMode.Contour;
            l.ContourColor = Color.ORANGE;
            l.Position = gridBase;
            l.SortKey.Value = 3;
            l.ContourSize = 3.0f;
            await world.CreateFadeIn(l, 0.5f);
            garbage.Add(l);
        }

        await Time.WaitSeconds(1.0f);
        return garbage;
    }

    var garbage = await tryExample(pairings1);
    await Animate.InterpF(x => {
        foreach (var s in garbage) {
            s.Color = Color.Lerp(s.Color, Color.TRANSPARENT, x);
        }
    }, 0.0f, 1.0f, 1.0f);
    foreach (var s in garbage) {
        world.Destroy(s);
    }

    await Time.WaitSeconds(1.0f);

    _ = Animate.Color(intersectionShapes[2], Color.YELLOW, col, 1.0f);
    await Animate.Color(intersectionShapes[4], col, Color.YELLOW, 1.0f);

    await Animate.Offset(caseLine, new Vector3(150.0f, 0.0f, 0.0f));
    await Time.WaitSeconds(1.0f);

    await tryExample(pairings2);

    await Time.WaitSeconds(1.0f);

    await Animate.Color(exampleShape, Color.TRANSPARENT, new Color(1.0f, 0.1f, 0.1f, 0.5f), 1.0f);

    await Time.WaitSeconds(2.0f);

    world.EndCapture();

    List<Task> tasks = new List<Task>();
    foreach (dynamic s in allEntities) {
        tasks.Add( Animate.InterpF(x => {
            s.Color = Color.Lerp(s.Color, Color.TRANSPARENT, x);
            if (s is Shape shape) {
                shape.ContourColor = Color.Lerp(s.ContourColor, Color.TRANSPARENT, x);
            }
        }, 0.0f, 1.0f, 1.0f));
    }
    await Task.WhenAll(tasks);
    foreach (dynamic s in allEntities) {
        if (!s.ManagedLifetime) {
            world.Destroy(s);
        }
    }

    _ = Animate.ChangeText(titleText, "3D internal ambiguity");
    await Animate.InterpF(x => {
        titleText.Anchor = new Vector2(x, titleText.Anchor.y);
    }, -0.3f, 0.0f, 1.0f);
}

async Task<Line3D> AnimateGrid(Vector3 min, float step, int size, Color color) {
    _ = Animate.ChangeText(titleText, "Cube generation: intersection points");

    var showCube = new Cube();
    showCube.Color = Color.WHITE;
    showCube.Scale = new Vector3(3.0f);
    world.CreateInstantly(showCube);
    await Animate.InterpF(x => {
        showCube.Color = Color.Lerp(Color.TRANSPARENT, Color.WHITE, x);
        showCube.Outline = Color.Lerp(Color.TRANSPARENT, Color.BLACK, x);
    }, 0.0f, 1.0f, 1.0f);

    await Time.WaitSeconds(2.0f); 

    var grid = new Line3D(mode: MeshVertexMode.Segments);
    grid.Color = color;
    grid.Width = 3.0f;
    world.CreateInstantly(grid);

    List<Vector3> vertices = new List<Vector3>();
    for (int z = 0; z < size+1; z++) {
        for (int i = 0; i < size+1; i++) {
            vertices.Add(min + new Vector3(i*step, z*step, 0.0f));
            vertices.Add(min + new Vector3(i*step, z*step, size*step));
            vertices.Add(min + new Vector3(  0.0f, z*step, i*step));
            vertices.Add(min + new Vector3(size*step, z*step, i*step));

            if (z != 0) {
                for (int j = 0; j < size+1; j++) {
                    vertices.Add(min + new Vector3(i*step, (z-1)*step, j*step));
                    vertices.Add(min + new Vector3(i*step, z*step, j*step));
                }
            }
            grid.Vertices = vertices.ToArray();
        }
        await Time.WaitSeconds(0.26f);
    }

    Sphere[,,] spheres = new Sphere[size+1, size+1, size+1];
    for (int i = 0; i < size+1; i++)
    for (int j = 0; j < size+1; j++)
    for (int k = 0; k < size+1; k++) {
        var sphere = new Sphere(0.05f);
        sphere.Color = Color.ORANGE;
        sphere.Position = min + new Vector3(i*step, j*step, k*step);
        spheres[i, j, k] = sphere;
        world.CreateInstantly(sphere);
        await Time.WaitSeconds(0.01f);
    }

    Func<Vector3, float> tinyCube = pos => {
        return sdBox(pos + new Vector3(float.Epsilon), new Vector3(1.5f));
    };

    Func<Vector3, Vector3> tinyCubeNormal = pos => {
        // numerical gradient
        float eps = 0.001f;
        var n = new Vector3(
            tinyCube(pos + new Vector3(eps, 0.0f, 0.0f)) - tinyCube(pos - new Vector3(eps, 0.0f, 0.0f)),
            tinyCube(pos + new Vector3(0.0f, eps, 0.0f)) - tinyCube(pos - new Vector3(0.0f, eps, 0.0f)),
            tinyCube(pos + new Vector3(0.0f, 0.0f, eps)) - tinyCube(pos - new Vector3(0.0f, 0.0f, eps))
        ).Normalized;
        return n;
    };

    foreach (var s in spheres) {
        var sample = tinyCube(s.Position);
        if (sample < 0.0f) {
            s.Color = Color.WHITE;
        } else {
            s.Color = Color.BLACK;
        }
    }

    await Time.WaitSeconds(1.0f);

    await Animate.InterpF(x => {
        showCube.Color = Color.Lerp(showCube.Color, Color.TRANSPARENT, x);
        showCube.Outline = Color.Lerp(showCube.Outline, Color.TRANSPARENT, x);
    }, 0.0f, 1.0f, 1.0f);

    world.Destroy(showCube);

    await Time.WaitSeconds(1.0f);

    float[,,] samples = new float[size+1, size+1, size+1];
    Vector3[,,] normals = new Vector3[size+1, size+1, size+1];
    Vector3[,,] positions = new Vector3[size+1, size+1, size+1];
    for (int i = 0; i < size+1; i++)
    for (int j = 0; j < size+1; j++)
    for (int k = 0; k < size+1; k++) {
        samples[i, j, k] = tinyCube(min + new Vector3(i*step, j*step, k*step));
        normals[i, j, k] = tinyCubeNormal(min + new Vector3(i*step, j*step, k*step));
        positions[i, j, k] = min + new Vector3(i*step, j*step, k*step);
    }

    List<(Vector3 p, Vector3 n)> intersections = new();
    for (int i = 0; i < size+1; i++)
    for (int j = 0; j < size+1; j++) {
        var s1 = min + new Vector3(i*step, j*step, 0.0f);
        var e1 = min + new Vector3(i*step, j*step, size*step);
        var s2 = min + new Vector3(0.0f, j*step, i*step);
        var e2 = min + new Vector3(size*step, j*step, i*step);
        var s3 = min + new Vector3(i*step, 0.0f, j*step);
        var e3 = min + new Vector3(i*step, size*step, j*step);

        var intersections1 = GetIntersections((s1, e1), tinyCube, tinyCubeNormal, steps: 512);
        var intersections2 = GetIntersections((s2, e2), tinyCube, tinyCubeNormal, steps: 512);
        var intersections3 = GetIntersections((s3, e3), tinyCube, tinyCubeNormal, steps: 512);
        intersections.AddRange(intersections1.Select(x => (x.pos, x.normal)));
        intersections.AddRange(intersections2.Select(x => (x.pos, x.normal)));
        intersections.AddRange(intersections3.Select(x => (x.pos, x.normal)));
    }


    List<Sphere> sphereList = new List<Sphere>();
    foreach (var (p, n) in intersections) {
        var s = new Sphere(0.03f);
        s.Position = p;
        s.Color = Color.YELLOW;
        sphereList.Add(s);
        world.CreateInstantly(s);
        await Time.WaitSeconds(0.01f);
    }
    await Time.WaitSeconds(1.0f);

    _ = Animate.ChangeText(titleText, "Cube generation: marching cubes result");

    var cmsCubeLines = CubicalMarchingSquares(samples, step, min);
    cmsCubeLines.Color = Color.ORANGE;
    await world.CreateFadeIn(cmsCubeLines, 0.5f);

    await Time.WaitSeconds(4.0f);

    await Animate.InterpF(x => {
        cmsCubeLines.Color = Color.Lerp(cmsCubeLines.Color, Color.TRANSPARENT, x);
    }, 0.0f, 1.0f, 1.0f);
    world.Destroy(cmsCubeLines);

    await Time.WaitSeconds(1.0f);

    var normalLines = new Line3D();
    normalLines.Color = Color.GREEN;
    normalLines.Vertices = intersections.SelectMany(i => new Vector3[] { i.p, i.p + 0.3f*i.n }).ToArray();
    normalLines.Width = 6.0f;
    await world.CreateFadeIn(normalLines, 0.5f);

    async Task hideGrid(bool hide) {
        await Animate.InterpF(x => {
            grid.Color = Color.Lerp(grid.Color, hide ? grid.Color.WithA(0) : grid.Color.WithA(0.3f), x);
            foreach (var s in spheres) {
                s.Color = Color.Lerp(s.Color, hide ? s.Color.WithA(0.0f) : s.Color.WithA(1.0f), x);
            }
            foreach (var s in sphereList) {
                s.Color = Color.Lerp(s.Color, hide ? s.Color.WithA(0.0f) : s.Color.WithA(1.0f), x);
            }
            normalLines.Color = Color.Lerp(normalLines.Color, hide ? normalLines.Color.WithA(0.0f) : normalLines.Color.WithA(1.0f), x);
        }, 0.0f, 1.0f, 1.0f);
    }

    // show individual quad

    await Time.WaitSeconds(3.0f);

    Line3D[] quads = new Line3D[6];
    Vector3 quadOffset = new Vector3(-2f, 1.0f, 0.0f);

    (List<Sphere> l, Color c)[] cornerSpheres = new (List<Sphere>, Color)[8];
    for (int i = 0; i < 8; i++) {
        cornerSpheres[i] = (new List<Sphere>(), Color.BLACK);
    }

    /*void setCornerColor(int corner, Color c) {
        foreach (var s in cornerSpheres[corner].l) {
            s.Color = c;
        }
    }*/

    for (int i = 0; i < 6; i++) {
        var q = new Line3D();
        q.Vertices =  lineQuadVertices;
        q.Color = Color.RED.WithA(0.4f);
        q.Width = 4.1f;

        quads[i] = q;
        q.Position = quadOffset + CMS.quadOffsets[i];
        _ = world.CreateFadeIn(q, 0.5f);

        for (int j = 0; j < 4; j++) {
            var s = new Sphere(0.051f);
            s.ParentId.Value = q.Id;
            s.Position = new Vector3(faceQuadPositions[i, j], 0.0f);
            var corner = CMS.faceCubeCorners[i, j];
            //var c = cornerColors[CMS.faceCubeCorners[i, j]];
            if (corner == 1 || corner == 5) {
                s.Color = Color.WHITE;
            } else {
                s.Color = Color.BLACK;
            }
            _ = world.CreateFadeIn(s, 0.5f);
            cornerSpheres[CMS.faceCubeCorners[i, j]].l.Add(s);
            cornerSpheres[CMS.faceCubeCorners[i, j]].c = s.Color;
        }
    }

    quads[5].ParentId.Value = quads[1].Id;
    quads[5].Position = new Vector3(1.0f, 0.0f, 0.0f);
    Vector3[] ps = quads.Select(q => q.Position).ToArray();
    await FoldAll(quadOffset, ps, quads, 0.0f, 0.0f, MathF.PI/2.0f);

    var iss = new Sphere[8];
    for (int i = 0; i < 8; i++) {
        var s = world.Clone(sphereList[0]);
        s.Radius = 0.031f;
        s.Color = s.Color.WithA(1.0f);
        iss[i] = s;
    }
    var issN = new Line3D[8];
    var issT = new Line3D[8];
    for (int i = 0; i < 8; i++) {
        var l = new Line3D();
        l.Color = Color.GREEN;
        l.Width = 6.0f;
        issN[i] = l;
    }

    (VisualEntity3D, Vector3, Vector3, bool)[] iPts = new (VisualEntity3D, Vector3, Vector3, bool)[8] {
        (quads[0], new Vector3(0.5f, 0.0f, 0.0f), new Vector3(-1.0f, 0.0f, 0.0f), false),
        (quads[5], new Vector3(0.5f, 0.0f, 0.0f), new Vector3( 1.0f, 0.0f, 0.0f), true),
        (quads[1], new Vector3(1.0f, 0.5f, 0.0f), new Vector3( 0.0f, 1.0f, 0.0f), true),
        (quads[1], new Vector3(0.0f, 0.5f, 0.0f), new Vector3( 0.0f, 1.0f, 0.0f), false),
        (quads[4], new Vector3(0.5f, 0.0f, 0.0f), new Vector3(-1.0f, 0.0f, 0.0f), false),
        (quads[0], new Vector3(1.0f, 0.5f, 0.0f), new Vector3( 0.0f, 1.0f, 0.0f), true),
        (quads[4], new Vector3(0.5f, 1.0f, 0.0f), new Vector3(-1.0f, 0.0f, 0.0f), true),
        (quads[5], new Vector3(0.0f, 0.5f, 0.0f), new Vector3( 0.0f, 1.0f, 0.0f), false),
    };

    for (int i = 0; i < 8; i++) {
        iss[i].ParentId.Value = iPts[i].Item1.Id;
        iss[i].Position = iPts[i].Item2;
        issN[i].ParentId.Value = iPts[i].Item1.Id;
        issN[i].Position = iPts[i].Item2;
        issN[i].Vertices = new Vector3[] { Vector3.ZERO, 0.3f*iPts[i].Item3 };
        _ = world.CreateInstantly(iss[i]);
        _ = world.CreateInstantly(issN[i]);
    }

    _ = Animate.ChangeText(titleText, "Cube generation: 2D marching squares with sharp features");

    var cam = world.ActiveCamera;
    var camOrigRot = cam.Rotation;
    var camOrigPos = cam.Position;

    var targetPosition = cam.Position + new Vector3(-4.0f, 0.0f, 9.0f);
    var targetLookAt = min + new Vector3(0.5f, 3.5f, 1.5f);
    var toTarget = targetLookAt - targetPosition;
    var targetRot = Quaternion.LookRotation(toTarget, Vector3.UP);
    var startRot = cam.Rotation;
    var startPos = cam.Position;
    await Animate.InterpF(x => {
        cam.Position = Vector3.Lerp(startPos, targetPosition, x);
        cam.Rotation = Quaternion.Slerp(startRot, targetRot, x);
    }, 0.0f, 1.0f, 1.0f);

    await hideGrid(true);


    await FoldAll(quadOffset, ps, quads, 1.0f, MathF.PI/2.0f, 0.0f);

    // separete quads
    foreach (var q in quads) {
        if (q == quads[0]) continue;
        var d = q.Position - quads[0].Position;
        if (q == quads[5]) d = Vector3.RIGHT;
        d.z = 0.0f;
        d = d.Normalized;
        _ = Animate.Offset(q, d * 0.5f);
    }
    await Time.WaitSeconds(1.0f);


    await Time.WaitSeconds(1.0f);

    for (int i = 0; i < 8; i++) {
        issT[i] = world.Clone(issN[i]);
        var dif = issN[i].Vertices[1] - issN[i].Vertices[0];
        var rot = Quaternion.AngleAxis((iPts[i].Item4 ? 1.0f : -1.0f) * MathF.PI/2.0f, Vector3.FORWARD);
        issT[i].Vertices = new Vector3[] { 
            issN[i].Vertices[0],
            issN[i].Vertices[0] + rot * dif
        };
        issT[i].Color = Color.MAGENTA;
        _ = world.CreateFadeIn(issT[i], 0.5f);
    }

    await Time.WaitSeconds(2.0f);

    int[] sharpFeatures = new int[] {0, 5};
    Sphere[] sharpSpheres = new Sphere[sharpFeatures.Length];
    foreach (var i in sharpFeatures) {
        var s = new Sphere(0.03f);
        s.Color = Color.ORANGE;
        s.Position = new Vector3(0.5f, 0.5f, 0.0f);
        s.ParentId.Value = quads[i].Id;
        _ = world.CreateFadeIn(s, 0.5f);
        sharpSpheres[Array.IndexOf(sharpFeatures, i)] = s;
    }

    (Vector2, Vector2)[][] lines = new (Vector2, Vector2)[6][] {
        new (Vector2, Vector2)[] {
            (new Vector2(0.5f, 0.0f), new Vector2(0.5f, 0.5f)),
            (new Vector2(0.5f, 0.5f), new Vector2(1.0f, 0.5f)),
        },
        new (Vector2, Vector2)[] {
            (new Vector2(0.0f, 0.5f), new Vector2(1.0f, 0.5f)),
        },
        new (Vector2, Vector2)[] {
        },
        new (Vector2, Vector2)[] {
        },
        new (Vector2, Vector2)[] {
            (new Vector2(0.5f, 0.0f), new Vector2(0.5f, 1.0f)),
        },
        new (Vector2, Vector2)[] {
            (new Vector2(0.5f, 0.0f), new Vector2(0.5f, 0.5f)),
            (new Vector2(0.5f, 0.5f), new Vector2(0.0f, 0.5f)),
        },
    };

    await Time.WaitSeconds(1.0f);

    List<Line3D> cSharpLines = new List<Line3D>();
    for (int i = 0; i < 6; i++) {
        foreach (var (p1, p2) in lines[i]) {
            var l = new Line3D();
            l.Vertices = new Vector3[] {
                new Vector3(p1.x, p1.y, 0.0f),
                new Vector3(p2.x, p2.y, 0.0f),
            };
            l.Color = Color.YELLOW;
            l.Width = 6.0f;
            l.ParentId.Value = quads[i].Id;
            _ = world.CreateFadeIn(l, 0.5f);
            cSharpLines.Add(l);
        }
    }

    await Time.WaitSeconds(1.0f);

    // fade out normals and tangents
    await Animate.InterpF(x => {
        foreach (var l in issN) {
            l.Color = Color.Lerp(l.Color, Color.TRANSPARENT, x);
        }
        foreach (var l in issT) {
            l.Color = Color.Lerp(l.Color, Color.TRANSPARENT, x);
        }
        foreach (var iss in iss) {
            iss.Color = Color.Lerp(iss.Color, Color.TRANSPARENT, x);
        }
    }, 0.0f, 1.0f, 1.0f);
    // destroy normals and tangents
    world.Destroy(issN);
    world.Destroy(issT);
    world.Destroy(iss);

    await Time.WaitSeconds(1.0f);

    // put quads back together
    List<Task> foldTasks = new ();
    foreach (var q in quads) {
        if (q == quads[0]) continue;
        var d = q.Position - quads[0].Position;
        if (q == quads[5]) d = Vector3.RIGHT;
        d.z = 0.0f;
        d = d.Normalized;
        foldTasks.Add(Animate.Offset(q, -d * 0.5f));
    }

    _ = Animate.ChangeText(titleText, "Cube generation: folding 2D segments");

    await Task.WhenAll(foldTasks);
    // fold quads back into cube
    await FoldAll(quadOffset, ps, quads, 1.0f, 0.0f, MathF.PI/2.0f);

    await Time.WaitSeconds(1.0f);

    List<Vector3>[,,] cellVertices = new List<Vector3>[size, size, size];
    for (int i = 0; i < size; i++)
    for (int j = 0; j < size; j++)
    for (int k = 0; k < size; k++) {
        cellVertices[i, j, k] = new List<Vector3>();
    }

    // fade out quads
    async Task hideQuadsNStuff() {
        await Animate.InterpF(x => {
            foreach (var q in quads) {
                q.Color = Color.Lerp(q.Color, Color.TRANSPARENT, x);
            }
            foreach (var sl in cSharpLines) {
                sl.Color = Color.Lerp(sl.Color, Color.TRANSPARENT, x);
            }
            foreach (var s in sharpSpheres) {
                s.Color = Color.Lerp(s.Color, Color.TRANSPARENT, x);
            }
            foreach (var s in cornerSpheres.SelectMany(x => x.l)) {
                s.Color = Color.Lerp(s.Color, Color.TRANSPARENT, x);
            }
        }, 0.0f, 1.0f, 1.0f);
        world.Destroy(quads);
        world.Destroy(cSharpLines.ToArray());
        world.Destroy(sharpSpheres);
        world.Destroy(cornerSpheres.SelectMany(x => x.l).ToArray());
    }
    _ = hideQuadsNStuff();

    async Task moveCam() {
        await Animate.InterpF(x => {
            cam.Position = Vector3.Lerp(startPos, targetPosition, x);
            cam.Rotation = Quaternion.Slerp(startRot, targetRot, x);
        }, 1.0f, 0.0f, 1.0f);
        await Animate.OrbitAndLookAt(cam, Vector3.UP, Vector3.ZERO, 720.0f, 20.0f);
    }

    _ = Animate.ChangeText(titleText, "Cube generation: rest of the segments");

    var camMoveTask = moveCam();
    await hideGrid(false);

    cmsCubeLines = await CubicalMarchingSquares2(positions, step, tinyCube, tinyCubeNormal, stepWait: 0.1f, cellVertices: cellVertices);

    var cornerVerts = cellVertices[0, 3, 0].ToArray();
    var cornerLines = new Line3D();
    cornerLines.Color = Color.YELLOW;
    cornerLines.Vertices = cornerVerts;
    cornerLines.Width = 6.0f;
    world.CreateInstantly(cornerLines);

    await camMoveTask;

    await Animate.InterpF(x => {
        cmsCubeLines.Color = Color.Lerp(cmsCubeLines.Color, Color.TRANSPARENT, x);
    }, 0.0f, 1.0f, 1.0f);
    world.Destroy(cmsCubeLines);


    targetPosition = cam.Position + new Vector3(-4.0f, 0.0f, 9.0f);
    targetLookAt = min + new Vector3(0.5f, 3.5f, 0.5f);
    toTarget = targetLookAt - targetPosition;
    targetRot = Quaternion.LookRotation(toTarget, Vector3.UP);

    //_ = hideGrid(true);

    (Vector3, Vector3)[] cornerNormals = new (Vector3, Vector3)[3] {
        (new Vector3(1.0f, 3.0f, 0.5f), new Vector3(1.0f, 3.0f, 0.2f)),
        (new Vector3(0.5f, 3.0f, 1.0f), new Vector3(0.2f, 3.0f, 1.0f)),
        (new Vector3(1.0f, 3.5f, 1.0f), new Vector3(1.0f, 3.8f, 1.0f)),
    };

    _ = Animate.ChangeText(titleText, "Cube generation: sharp feature (full rank)");

    // move camera back
    await Animate.InterpF(x => {
        cam.Position = Vector3.Lerp(startPos, targetPosition, x);
        cam.Rotation = Quaternion.Slerp(startRot, targetRot, x);
    }, 0.0f, 1.0f, 1.0f);

    var cornerLines2 = new Line3D[3];
    for (int i = 0; i < 3; i++) {
        var cornerLine = new Line3D();
        cornerLine.Color = Color.GREEN;
        cornerLine.Vertices = new Vector3[] { cornerNormals[i].Item1 + min + new Vector3(0.001f), cornerNormals[i].Item2 + min + new Vector3(0.001f) };
        cornerLine.Width = 6.0f;
        world.CreateInstantly(cornerLine);
        cornerLines2[i] = cornerLine;
    }

    // fade out normals and tangents
    await Animate.InterpF(x => {
        normalLines.Color = Color.Lerp(normalLines.Color, Color.TRANSPARENT, x);
    }, 0.0f, 1.0f, 1.0f);

    Func<int, Color> planeColor = i => {
        return i switch {
            0 => Color.RED,
            1 => Color.GREEN,
            2 => Color.BLUE,
            _ => Color.WHITE,
        };
    };

    await Animate.InterpF(x => {
        for (int i = 0; i < 3; i++) {
            var baseC = planeColor(i);
            cornerLines2[i].Color = Color.Lerp(Color.GREEN, 1.5f*baseC, x);
        }
    }, 0.0f, 1.0f, 1.0f);
    await Animate.InterpF(x => {
        for (int i = 0; i < 3; i++) {
            var baseC = planeColor(i);
            cornerLines2[i].Color = Color.Lerp(1.5f*baseC, baseC, x);
        }
    }, 0.0f, 1.0f, 1.0f);

    (Vector3, Vector3, Vector3, Vector3)[] planeQuadVertices = new (Vector3, Vector3, Vector3, Vector3)[] {
        (new Vector3(0.5f, 3.0f, 0.5f), new Vector3(1.0f, 3.0f, 0.5f), new Vector3(1.0f, 3.5f, 0.5f), new Vector3(0.5f, 3.5f, 0.5f)),
        (new Vector3(0.5f, 3.0f, 1.0f), new Vector3(0.5f, 3.0f, 0.5f), new Vector3(0.5f, 3.5f, 0.5f), new Vector3(0.5f, 3.5f, 1.0f)),
        (new Vector3(0.5f, 3.5f, 1.0f), new Vector3(0.5f, 3.5f, 0.5f), new Vector3(1.0f, 3.5f, 0.5f), new Vector3(1.0f, 3.5f, 1.0f)),
    };

    Quad[] planeQuads = new Quad[3];
    for (int i = 0; i < 3; i++) {
        var (p1, p2, p3, p4) = planeQuadVertices[i];
        var q = new Quad();
        q.Vertices = [p1 + min, p2 + min, p3 + min, p4 + min];
        q.Color = planeColor(i).WithA(0.0f);
        q.Outline = Color.BLACK;
        world.CreateInstantly(q);
        await Animate.InterpF(x => {
            q.Color = Color.Lerp(q.Color, planeColor(i).WithA(1.0f), x);
            q.Outline = Color.Lerp(Color.BLACK.WithA(0.0f), Color.BLACK.WithA(1.0f), x);
        }, 0.0f, 1.0f, 1.0f);
        planeQuads[i] = q;
    }

    await Time.WaitSeconds(1.0f);

    List<Sphere> sharpSphereList = new ();

    var solved = new Sphere(0.03f);
    solved.Color = 1.5f*Color.RED;
    solved.Position = new Vector3(0.5f, 3.5f, 0.5f) + min;
    await world.CreateFadeIn(solved, 0.5f);
    await Animate.Color(solved, Color.RED, 0.5f);
    sharpSphereList.Add(solved);

    await Time.WaitSeconds(1.0f);

    await Animate.InterpF(x => {
        foreach (var q in planeQuads) {
            q.Color = Color.Lerp(q.Color, Color.TRANSPARENT, x);
            q.Outline = Color.Lerp(q.Outline, Color.TRANSPARENT, x);
        }
    }, 0.0f, 1.0f, 1.0f);
    world.Destroy(planeQuads);

    Vector3[] fanVertices = new Vector3[6] {
        new Vector3(0.5f, 3.0f, 0.5f),
        new Vector3(1.0f, 3.0f, 0.5f),
        new Vector3(1.0f, 3.5f, 0.5f),
        new Vector3(1.0f, 3.5f, 1.0f),
        new Vector3(0.5f, 3.5f, 1.0f),
        new Vector3(0.5f, 3.0f, 1.0f),
    };
    var fanColor = new Color("#baac7d"); 
    var mesh = new Mesh();
    List<Vector3> fanV = new ();
    List<uint> fanI = new ();
    // create fan to solved.Position
    for (int i = 0; i < 6; i++) {
        fanV.Add(fanVertices[i] + min);
    }
    fanV.Add(solved.Position);
    for (int i = 0; i < 6; i++) {
        fanI.Add((uint)i);
        fanI.Add(6);
        fanI.Add((uint)(i+1)%6);
    }
    mesh.Vertices = fanV.ToArray();
    mesh.Indices = fanI.ToArray();
    mesh.Color = fanColor;
    await world.CreateFadeIn(mesh, 0.5f);

    await Time.WaitSeconds(1.0f);

    _ = Animate.ChangeText(titleText, "Cube generation: no sharp feature");

    targetLookAt = min + new Vector3(0.5f, 2.5f, 1.5f);
    toTarget = targetLookAt - targetPosition;
    targetRot = Quaternion.LookRotation(toTarget, Vector3.UP);
    startRot = cam.Rotation;
    await Animate.InterpF(x => {
        cam.Rotation = Quaternion.Slerp(startRot, targetRot, x);
    }, 0.0f, 1.0f, 1.0f);

    foreach (var cl in cornerLines2) {
        _ = Animate.InterpF(x => {
            cl.Color = Color.Lerp(cl.Color, Color.TRANSPARENT, x);
        }, 0.0f, 1.0f, 1.0f);
    }
    await Time.WaitSeconds(1.0f);
    world.Destroy(cornerLines2);

    cornerNormals = new (Vector3, Vector3)[4] {
        (new Vector3(0.5f, 2.0f, 1.0f), new Vector3(0.2f, 2.0f, 1.0f)),
        (new Vector3(0.5f, 3.0f, 1.0f), new Vector3(0.2f, 3.0f, 1.0f)),
        (new Vector3(0.5f, 3.0f, 2.0f), new Vector3(0.2f, 3.0f, 2.0f)),
        (new Vector3(0.5f, 2.0f, 2.0f), new Vector3(0.2f, 2.0f, 2.0f)),
    };
    cornerLines2 = new Line3D[4];
    for (int i = 0; i < 4; i++) {
        var cornerLine = new Line3D();
        cornerLine.Color = 1.5f*Color.GREEN;
        cornerLine.Vertices = new Vector3[] { cornerNormals[i].Item1 + min + new Vector3(0.001f), cornerNormals[i].Item2 + min + new Vector3(0.001f) };
        cornerLine.Width = 6.0f;
        _ = world.CreateFadeIn(cornerLine, 0.5f);
        cornerLines2[i] = cornerLine;
    }
    Line3D cornerContour = new Line3D();
    cornerContour.Color = Color.YELLOW;
    cornerContour.Width = 6.0f;
    cornerContour.Vertices = new Vector3[] {
        cornerNormals[3].Item1 + min + new Vector3(0.001f),
        cornerNormals[0].Item1 + min + new Vector3(0.001f),
        cornerNormals[0].Item1 + min + new Vector3(0.001f),
        cornerNormals[1].Item1 + min + new Vector3(0.001f),
        cornerNormals[1].Item1 + min + new Vector3(0.001f),
        cornerNormals[2].Item1 + min + new Vector3(0.001f),
        cornerNormals[2].Item1 + min + new Vector3(0.001f),
        cornerNormals[3].Item1 + min + new Vector3(0.001f),
    };
    _ = world.CreateFadeIn(cornerContour, 0.5f);
    await Animate.InterpF(x => {
        foreach (var cl in cornerLines2) {
            cl.Color = Color.Lerp(cl.Color, Color.GREEN, x);
        }
    }, 0.0f, 1.0f, 1.0f);

    var centerVert = new Vector3(0.5f, 2.5f, 1.5f);
    var centerSphere = new Sphere(0.03f);
    centerSphere.Color = 1.5f*Color.RED;
    centerSphere.Position = centerVert + min;
    await world.CreateFadeIn(centerSphere, 0.5f);
    await Animate.Color(centerSphere, Color.RED, 0.5f);
    sharpSphereList.Add(centerSphere);

    await Time.WaitSeconds(1.0f);

    Mesh fanMesh2 = new Mesh();
    fanV.Clear();
    fanI.Clear();
    // create fan to center
    for (int i = 0; i < 4; i++) {
        fanV.Add(cornerNormals[i].Item1 + min);
    }
    fanV.Add(centerVert + min);
    for (int i = 0; i < 4; i++) {
        fanI.Add((uint)(i+1)%4);
        fanI.Add(4);
        fanI.Add((uint)(i+2)%4);
    }
    fanMesh2.Vertices = fanV.ToArray();
    fanMesh2.Indices = fanI.ToArray();
    fanMesh2.Color = fanColor;
    fanMesh2.Outline = Color.BLACK;
    await world.CreateFadeIn(fanMesh2, 0.5f);

    await Time.WaitSeconds(1.0f);

    _ = Animate.ChangeText(titleText, "Cube generation: sharp feature (degenerate)");

    targetLookAt = min + new Vector3(0.5f, 3.5f, 1.5f);
    toTarget = targetLookAt - targetPosition;
    targetRot = Quaternion.LookRotation(toTarget, Vector3.UP);
    startRot = cam.Rotation;

    async Task hideCornerLines2() {
        foreach (var cl in cornerLines2) {
            _ = Animate.InterpF(x => {
                cl.Color = Color.Lerp(cl.Color, Color.TRANSPARENT, x);
            }, 0.0f, 1.0f, 1.0f);
        }
        await Time.WaitSeconds(1.0f);
        world.Destroy(cornerLines2);
    }
    var hcl2 = hideCornerLines2();

    await Animate.InterpF(x => {
        cam.Rotation = Quaternion.Slerp(startRot, targetRot, x);
    }, 0.0f, 1.0f, 1.0f);

    var cornerNormals3 = new (Vector3, Vector3, Color)[] {
        (new Vector3(0.5f, 3.0f, 1.0f), new Vector3(0.2f, 3.0f, 1.0f), Color.RED),
        (new Vector3(0.5f, 3.0f, 2.0f), new Vector3(0.2f, 3.0f, 2.0f), Color.RED),
        (new Vector3(1.0f, 3.5f, 1.0f), new Vector3(1.0f, 3.8f, 1.0f), Color.GREEN),
        (new Vector3(1.0f, 3.5f, 2.0f), new Vector3(1.0f, 3.8f, 2.0f), Color.GREEN),
    };
    await hcl2;
    cornerLines2 = new Line3D[cornerNormals3.Length];
    for (int i = 0; i < cornerNormals3.Length; i++) {
        var cornerLine = new Line3D();
        cornerLine.Color = 1.5f*cornerNormals3[i].Item3;
        cornerLine.Vertices = new Vector3[] { cornerNormals3[i].Item1 + min + new Vector3(0.002f), cornerNormals3[i].Item2 + min + new Vector3(0.002f) };
        cornerLine.Width = 6.0f;
        _ = world.CreateFadeIn(cornerLine, 0.5f);
        cornerLines2[i] = cornerLine;
    }

    var cornerContour2 = new Line3D();
    cornerContour2.Color = Color.YELLOW;
    cornerContour2.Width = 6.0f;
    cornerContour2.Vertices = new Vector3[] {
        cornerNormals3[0].Item1 + min + new Vector3(0.002f),
        cornerNormals3[1].Item1 + min + new Vector3(0.002f),
        cornerNormals3[1].Item1 + min + new Vector3(0.002f),
        new Vector3(0.5f, 3.5f, 2.0f) + min + new Vector3(0.002f),
        new Vector3(0.5f, 3.5f, 2.0f) + min + new Vector3(0.002f),
        cornerNormals3[3].Item1 + min + new Vector3(0.002f),
        cornerNormals3[3].Item1 + min + new Vector3(0.002f),
        cornerNormals3[2].Item1 + min + new Vector3(0.002f),
        cornerNormals3[2].Item1 + min + new Vector3(0.002f),
        new Vector3(0.5f, 3.5f, 1.0f) + min + new Vector3(0.002f),
        new Vector3(0.5f, 3.5f, 1.0f) + min + new Vector3(0.002f),
        cornerNormals3[0].Item1 + min + new Vector3(0.002f),
    };
    _ = world.CreateFadeIn(cornerContour2, 0.5f);

    await Time.WaitSeconds(0.5f);
    await Animate.InterpF(x => {
        foreach (var (cl, cn) in cornerLines2.Zip(cornerNormals3, (cl, cn) => (cl, cn))) { 
            cl.Color = Color.Lerp(cl.Color, cn.Item3, x);
        }
    }, 0.0f, 1.0f, 1.0f);

    await Time.WaitSeconds(1.0f);

    var sharpCenter = new Sphere(0.03f);
    sharpCenter.Color = 1.5f*Color.RED;
    sharpCenter.Position = new Vector3(0.5f, 3.5f, 1.5f) + min;
    _ = world.CreateFadeIn(sharpCenter, 0.5f);
    _ = Animate.Color(sharpCenter, Color.RED, 0.5f);
    sharpSphereList.Add(sharpCenter);

    await Time.WaitSeconds(1.0f);

    Mesh fanMesh3 = new Mesh();
    fanV.Clear();
    fanI.Clear();
    fanV.Add(new Vector3(0.5f, 3.0f, 1.0f) + min);
    fanV.Add(new Vector3(0.5f, 3.0f, 2.0f) + min);
    fanV.Add(new Vector3(0.5f, 3.5f, 2.0f) + min);
    fanV.Add(new Vector3(0.5f, 3.5f, 1.0f) + min);

    fanV.Add(new Vector3(0.5f, 3.5f, 1.0f) + min);
    fanV.Add(new Vector3(0.5f, 3.5f, 2.0f) + min);
    fanV.Add(new Vector3(1.0f, 3.5f, 2.0f) + min);
    fanV.Add(new Vector3(1.0f, 3.5f, 1.0f) + min);

    fanV.Add(sharpCenter.Position);
    fanI.AddRange([0, 1, 8, 1, 2, 8, 2, 3, 8, 3, 0, 8,  4, 5, 8, 5, 6, 8, 6, 7, 8, 7, 4, 8]);
    fanMesh3.Vertices = fanV.ToArray();
    fanMesh3.Indices = fanI.ToArray();
    fanMesh3.Color = fanColor;
    fanMesh3.Outline = Color.BLACK;
    await world.CreateFadeIn(fanMesh3, 0.5f);

    await Time.WaitSeconds(1.0f);

    /*var fanMesh32 = world.Clone(fanMesh31);
    fanV.Clear();
    fanI.Clear();
    fanV.Add(new Vector3(0.5f, 3.5f, 1.0f) + min);
    fanV.Add(new Vector3(0.5f, 3.5f, 2.0f) + min);
    fanV.Add(new Vector3(1.0f, 3.5f, 2.0f) + min);
    fanV.Add(new Vector3(1.0f, 3.5f, 1.0f) + min);
    fanV.Add(sharpCenter1.Position);
    fanI.AddRange([0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4]);
    fanMesh32.Vertices = fanV.ToArray();
    fanMesh32.Indices = fanI.ToArray();
    fanMesh32.Color = Color.ORANGE;
    fanMesh32.Outline = Color.BLACK;
    await world.CreateFadeIn(fanMesh32, 0.5f);

    await Time.WaitSeconds(1.0f);*/

    _ = Animate.ChangeText(titleText, "Cube generation: done");

    var wholeMesh = DoCMS2(tinyCube, tinyCubeNormal, nmax: 0);
    wholeMesh.Color = fanColor;
    wholeMesh.Outline = Color.BLACK;
    async Task createMesh() {
        await world.CreateFadeIn(wholeMesh, 0.5f);
        await Animate.InterpF(x => {
            foreach(var cl in cornerLines2) {
                cl.Color = Color.Lerp(cl.Color, Color.TRANSPARENT, x);
            }
            cornerLines.Color = Color.Lerp(cornerLines.Color, Color.TRANSPARENT, x);
            cornerContour.Color = Color.Lerp(cornerContour.Color, Color.TRANSPARENT, x);
            cornerContour2.Color = Color.Lerp(cornerContour2.Color, Color.TRANSPARENT, x);
            /*fanMesh31.Color = Color.Lerp(fanMesh31.Color, Color.TRANSPARENT, x);
            fanMesh31.Outline = Color.Lerp(fanMesh31.Outline, Color.TRANSPARENT, x);
            fanMesh32.Color = Color.Lerp(fanMesh32.Color, Color.TRANSPARENT, x);
            fanMesh32.Outline = Color.Lerp(fanMesh32.Outline, Color.TRANSPARENT, x);*/
            fanMesh2.Color = Color.Lerp(fanMesh2.Color, Color.TRANSPARENT, x);
            fanMesh2.Outline = Color.Lerp(fanMesh2.Outline, Color.TRANSPARENT, x);
            solved.Color = Color.Lerp(solved.Color, Color.TRANSPARENT, x);
            mesh.Color = Color.Lerp(mesh.Color, Color.TRANSPARENT, x);
            foreach (var s in sharpSphereList) {
                s.Color = Color.Lerp(s.Color, Color.TRANSPARENT, x);
            }
        }, 0.0f, 1.0f, 1.0f);
        world.Destroy(cornerLines);
        world.Destroy(cornerContour);
        world.Destroy(cornerContour2);
        world.Destroy(cornerLines2);
        world.Destroy(fanMesh3);
        //world.Destroy(fanMesh32);
        world.Destroy(fanMesh2);
        world.Destroy(solved);
        world.Destroy(mesh);
        world.Destroy(sharpSphereList.ToArray());
    }
    _ = createMesh();

    startRot = cam.Rotation;
    await Animate.InterpF(x => {
        cam.Rotation = Quaternion.Slerp(startRot, camOrigRot, x);
        cam.Position = Vector3.Lerp(targetPosition, camOrigPos, x);
    }, 0.0f, 1.0f, 1.0f);
    await Animate.OrbitAndLookAt(cam, Vector3.UP, Vector3.ZERO, 360.0f, 5.0f);

    _ = Animate.ChangeText(titleText, "");

    // hide all
    _ = hideGrid(true);
    await Animate.InterpF(x => {
        wholeMesh.Color = Color.Lerp(wholeMesh.Color, Color.TRANSPARENT, x);
        wholeMesh.Outline = Color.Lerp(wholeMesh.Outline, Color.TRANSPARENT, x);
    }, 0.0f, 1.0f, 1.0f);
    world.Destroy(wholeMesh);

    return grid;
}

List<Vector3> make3DGrid(Vector3 min, float step, int size) {
    List<Vector3> vertices = new List<Vector3>();
    for (int j = 0; j < size+1; j++)
    for (int i = 0; i < size+1; i++) {
        vertices.Add(min + new Vector3(i*step, j*step, 0.0f));
        vertices.Add(min + new Vector3(i*step, j*step, size*step));

        vertices.Add(min + new Vector3(i*step, 0.0f, j*step));
        vertices.Add(min + new Vector3(i*step, size*step, j*step));

        vertices.Add(min + new Vector3(0.0f, i*step, j*step));
        vertices.Add(min + new Vector3(size*step, i*step, j*step));
    }
    return vertices;
}

}
