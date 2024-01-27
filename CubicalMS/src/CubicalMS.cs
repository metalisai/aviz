using System;
using System.Linq;
using AnimLib;
using System.Collections.Generic;
using System.Threading.Tasks;
using CommunityToolkit.HighPerformance;

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
    settings.MaxLength = 600.0f; 
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

public static Vector3 EvaluateNormal(Vector3 pos) {
    //return sdTorusNormal(pos);
    // use gradient approximation
    float eps = 0.001f;
    float d = Evaluate2(pos);
    float dx = Evaluate2(pos + new Vector3(eps, 0.0f, 0.0f));
    float dy = Evaluate2(pos + new Vector3(0.0f, eps, 0.0f));
    float dz = Evaluate2(pos + new Vector3(0.0f, 0.0f, eps));
    return new Vector3(dx - d, dy - d, dz - d).Normalized;
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
    lines.Width = 4.0f;
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
            var sphere = new Sphere();
            sphere.Radius = 0.05f;
            sphere.Color = Color.ORANGE;
            sphere.Transform.Pos = worldP;
            //world.CreateInstantly(sphere);
            vertexPositions[vi] = worldP;

            vertices.Add(worldP);

            if (vi % 2 == 1) {

                var line = new Line3D();
                line.Color = cornerColors[face]; // DEBUG
                line.Vertices = [vertexPositions[vi-1], vertexPositions[vi]];
                line.Width = 4.0f;
                //world.CreateInstantly(line);
            }
        }
    }

    var lines = new Line3D();
    lines.Color = Color.ORANGE;
    lines.Vertices = vertices.ToArray();
    lines.Width = 4.0f;
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
            sphere.Transform.Pos = p;
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

public Mesh DoCMS() {
    List<(Vector3, Vector3)> intersectionsInternal = new();
    List<(Vector3, Vector3)> intersectionsFace = new();
    List<(Vector3, Vector3)> intersectionNormals = new();

    var tree = new CMSTree();
    var sw = new System.Diagnostics.Stopwatch();
    sw.Start();
    Func<(Vector3 s, Vector3 e), HermiteIntersection[]> getIntr = (pos) => {
        return GetIntersections(pos, Evaluate, EvaluateNormal);
    };
    Debug.Log($"Start CMS");
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
    intrLines.Width = 2.0f;
    //world.CreateInstantly(intrLines);

    var faceLines = new Line3D();
    faceLines.Color = Color.VIOLET;
    faceLines.Vertices = intersectionsFace.SelectMany(i => new Vector3[] { i.Item1, i.Item2 }).ToArray();
    faceLines.Width = 2.0f;
    //world.CreateInstantly(faceLines);

    var normalLines = new Line3D();
    normalLines.Color = Color.GREEN;
    normalLines.Vertices = intersectionNormals.SelectMany(i => new Vector3[] { i.Item1, i.Item1 + 0.3f*i.Item2 }).ToArray();
    normalLines.Width = 3.0f;
    //world.CreateInstantly(normalLines);

    //await Time.WaitSeconds(1.0f);
    //return;

    List<CMSCell> cells = new List<CMSCell>();
    tree.getLeafCells(tree.root, cells);
    foreach (var cell in cells) {
        if (cell.size == 2.0f) continue;
        //var c = world.Clone(cube);
        //c.Transform.Pos = cell.min;
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

Cube cube;

public async Task FoldQuad(VisualEntity3D q, Vector3 pos, Vector3 pivot, Vector3 axis, float startAngle, float endAngle, double duration) {
    Vector3 d = pos - pivot;
    await Animate.InterpF(x => {
        var rot = Quaternion.AngleAxis(x, axis);
        q.Transform.Rot = rot;
        var p = rot * d;
        q.Transform.Pos = pivot + p;
    }, startAngle, endAngle, duration);
}

async Task FoldAll(Vector3 quadOffset, Vector3[] ps, VisualEntity3D[] quads, double duration, float start, float end) {
    _ = FoldQuad(quads[1], ps[1], quadOffset + new Vector3(1.0f, 0.0f, 0.0f), Vector3.UP, start, end, duration);
    _ = FoldQuad(quads[2], ps[2], quadOffset + new Vector3(0.0f, 1.0f, 0.0f), -Vector3.RIGHT, start, end, duration);
    _ = FoldQuad(quads[3], ps[3], quadOffset + Vector3.ZERO, -Vector3.UP, start, end, duration);
    _ = FoldQuad(quads[4], ps[4], quadOffset + Vector3.ZERO, Vector3.RIGHT, start, end, duration);
    await FoldQuad(quads[5], ps[5], new Vector3(1.0f, 0.0f, 0.0f), Vector3.UP, start, end, duration);
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

public async Task Animation(World world, Animator animator) {
    Test();

    this.world = world;
    this.animator = animator;

    var cam = world.ActiveCamera;
    cam.Transform.Pos = new Vector3(0.0f, 0.0f, -5.0f);
    cam.ClearColor = new Color(0.035f, 0.03f, 0.05f, 1.0f);

    var c1 = new Sphere();
    c1.Radius = 0.05f;
    c1.Color = Color.YELLOW;
    c1.Transform.Pos = new Vector3(-2.5f, 0.0f, 0.5f);

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

    for (int i = 0; i < 6; i++) {
        var q = new Line3D();
        q.Vertices =  lineQuadVertices;
        q.Color = Color.BLACK;
        q.Width = 5.0f;

        quads[i] = q;
        q.Transform.Pos = quadOffset + CMS.quadOffsets[i];
        world.CreateInstantly(q);

        for (int j = 0; j < 4; j++) {
            var s = world.Clone(c1);
            s.Transform.parent = q.Transform;
            s.Transform.Pos = new Vector3(faceQuadPositions[i, j], 0.0f);
            var c = cornerColors[CMS.faceCubeCorners[i, j]];
            s.Color = c;
            world.CreateInstantly(s);
            cornerSpheres[CMS.faceCubeCorners[i, j]].l.Add(s);
            cornerSpheres[CMS.faceCubeCorners[i, j]].c = c;
        }
    }

    foreach (int i in Enumerable.Range(0, 8)) {
        setCornerColor(i, Color.BLACK);
    }

    quads[5].Transform.parent = quads[1].Transform;
    quads[5].Transform.Pos = new Vector3(1.0f, 0.0f, 0.0f);

    Vector3[] ps = quads.Select(q => q.Transform.Pos).ToArray();

    await FoldAll(quadOffset, ps, quads, 0.0f, 0.0f, MathF.PI/2.0f);

    Color exampleCubeColor = (0.5f*Color.ORANGE).WithA(0.5f);

    var exampleCube = new Cube();
    exampleCube.Color = exampleCubeColor;
    exampleCube.Transform.Pos = new Vector3(-17.0f, 0.0f, 0.0f);
    exampleCube.Transform.Scale = new Vector3(10.0f, 10.0f, 10.0f);
    world.CreateInstantly(exampleCube);

    await Animate.OrbitAndLookAt(cam.Transform, Vector3.UP, Vector3.ZERO, -20.0f, 1.0f);
    await Animate.Move(exampleCube.Transform, (-5.0f, 0.0f, 0.0f));
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
        s.Transform.Pos = intersections[i];
        s.Radius = 0.03f;
        s.Color = Color.YELLOW;
        intersectionSpheres[i] = s;
        _ = world.CreateFadeIn(s, 0.5f);
    }
    await Time.WaitSeconds(1.5f);

    await Animate.Move(exampleCube.Transform, (-50.0f, 0.0f, 0.0f));
    world.Destroy(exampleCube);

    var lines = new Line3D[5];
    lines[0] = await DrawLine(intersections[0], intersections[1], Color.YELLOW, 3.0f);
    lines[1] = await DrawLine(intersections[1], intersections[2], Color.YELLOW, 3.0f);
    lines[2] = await DrawLine(intersections[2], intersections[3], Color.YELLOW, 3.0f);
    lines[3] = await DrawLine(intersections[3], intersections[0], Color.YELLOW, 3.0f);
    lines[4] = await DrawLine(intersections[0], intersections[2], Color.YELLOW, 3.0f);

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

    await FoldAll(quadOffset, ps, quads, 1.0f, MathF.PI/2.0f, 0.0f);

    // separete quads
    foreach (var q in quads) {
        if (q == quads[0]) continue;
        var d = q.Transform.Pos - quads[0].Transform.Pos;
        if (q == quads[5]) d = Vector3.RIGHT;
        d.z = 0.0f;
        d = d.Normalized;
        _ = Animate.Offset(q.Transform, d * 0.5f);
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
        line.Transform.parent = quads[parents[i]].Transform;
        line.Color = Color.YELLOW;
        line.Width = 3.0f;
        _ = world.CreateFadeIn(line, 0.5f);
        ilines[i] = line;
    }
    await Time.WaitSeconds(1.0f);

    // put quads back together
    foreach (var q in quads) {
        if (q == quads[0]) continue;
        var d = q.Transform.Pos - quads[0].Transform.Pos;
        if (q == quads[5]) d = Vector3.RIGHT;
        d.z = 0.0f;
        d = d.Normalized;
        _ = Animate.Offset(q.Transform, -d * 0.5f);
    }
    await Time.WaitSeconds(1.0f);

    // fold quads
    await FoldAll(quadOffset, ps, quads, 1.0f, 0.0f, MathF.PI/2.0f);
    mesh.Color = Color.YELLOW;
    mesh.Outline = Color.YELLOW;
    await world.CreateFadeIn(mesh, 0.5f);

    await Animate.OrbitAndLookAt(cam.Transform, Vector3.UP, Vector3.ZERO, 20.0f, 1.0f);
    await Animate.Offset(cam.Transform, new Vector3(0.0f, 0.0f, -13.0f));

    var msLine = CubicalMarchingSquares();

    var orbitTask =  Animate.OrbitAndLookAt(cam.Transform, Vector3.UP, Vector3.ZERO, 720.0f, 15.0f);

    await world.CreateFadeIn(msLine, 1.0f);
    await Time.WaitSeconds(5.0f);
    await Animate.InterpF(x => {
        msLine.Color = Color.Lerp(Color.YELLOW, Color.TRANSPARENT, x);
    }, 0.0f, 1.0f, 0.5f);
    world.Destroy(msLine);
    await Time.WaitSeconds(0.5f);

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
    await Animate.Offset(cam.Transform, new Vector3(0.0f, 0.0f, 5.0f));

    cam.Transform.Pos = new Vector3(0.0f, 2.0f, cam.Transform.Pos.z);
    orbitTask = Animate.OrbitAndLookAt(cam.Transform, Vector3.UP, Vector3.ZERO, 720.0f, 20.0f);

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

    await AnimateAmbiguous2D();

    await AnimateAmbiguous3D();
}

async Task AnimateAmbiguous3D() {
    var allEntities = world.BeginCapture();

    var cam = world.ActiveCamera;
    cam.Transform.Pos = new Vector3(0.0f, 0.0f, -5.0f);
    cam.Transform.Rot = Quaternion.LookRotation(Vector3.FORWARD, Vector3.UP);

    var orbitTask = Animate.OrbitAndLookAt(cam.Transform, Vector3.UP, Vector3.ZERO, 720.0f, 20.0f);

    var offset1 = new Vector3(-1.5f, -0.5f, -0.5f); 
    var offset2 = new Vector3(1.5f, -0.5f, -0.5f); 
    // create 8 cube corners
    for (int i = 0; i < 8; i++) {
        var pos = CMS.cornerOffsets[i];
        var s = new Sphere();
        s.Radius = 0.05f;
        s.Color = (i == 0 || i == 6) ? new Color(0.5f, 0.5f, 0.5f, 1.0f) :Color.BLACK;
        s.Transform.Pos = pos + offset1;
        var s2 = world.Clone(s);
        s2.Transform.Pos = pos + offset2;
        world.CreateInstantly(s);
        world.CreateInstantly(s2);
    }

    var exampleMesh1a = new Mesh();
    exampleMesh1a.Color = new Color(1.0f, 0.05f, 0.05f, 1.0f);
    exampleMesh1a.Outline = Color.BLACK;
    exampleMesh1a.Transform.Pos = offset1;
    exampleMesh1a.Vertices = new Vector3[] {
        (0.56f, -0.58f, -0.35f),
        (0.1f, 0.6f, 0.1f),
        (0.2f, -0.5f, 0.6f),
        (0.2f, 0.2f, 0.3f),
        (-0.5f, 0.5f, 0.1f),
        (-1.0f, -0.5f, 0.55f), 
        (0.0f, -0.5f, -0.58f),
        (-1.0f, -0.5f, -0.53f), 
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
    exampleMesh1b.Transform.Pos = offset1 + new Vector3(1.0f, 1.0f, 1.0f);
    var rot = Quaternion.AngleAxis(MathF.PI, Vector3.FORWARD);
    rot = rot * Quaternion.AngleAxis(MathF.PI, Vector3.UP);
    exampleMesh1b.Transform.Rot = rot;
    world.CreateInstantly(exampleMesh1b);

    var exampleMesh2 = new Mesh();
    exampleMesh2.Color = new Color(1.0f, 0.05f, 0.05f, 1.0f);
    exampleMesh2.Outline = Color.BLACK;
    exampleMesh2.Transform.Pos = offset2;
    exampleMesh2.Vertices = new Vector3[] {
        (0.55f, 0.2f, 0.15f),
        (0.1f, 0.6f, 0.1f),
        (0.034f, 0.07f, 0.62f),

        (0.45f, 1.1f, 1.27f),
        (0.95f, 0.4f, 0.70f),
        (0.8f, 1.15f, 0.6f),

        (0.5f, 0.0f, 0.0f) - new Vector3(0.5f) - new Vector3(0.35f, -0.25f, -0.25f),
        (0.5f, 1.0f, 1.0f) + new Vector3(0.5f) + new Vector3(0.35f, -0.25f, -0.25f),
    };
    exampleMesh2.Indices = new uint[] { 
        0, 5, 1,
        0, 5, 4,
        1, 3, 5,
        1, 3, 2,
        0, 2, 4,
        2, 4, 3,

        0, 6, 1,
        0, 2, 6,
        1, 6, 2,

        3, 4, 7,
        3, 5, 7,
        4, 5, 7,
    };
    world.CreateInstantly(exampleMesh2);

    var cubeLines = new Line3D();
    cubeLines.Color = Color.BLACK;
    cubeLines.Width = 2.0f;
    Vector3[] vs = new Vector3[24];
    for (int i = 0; i < 12; i++) {
        var (c1, c2) = CMS.cubeEdgeCorners[i];
        vs[2*i] = CMS.cornerOffsets[c1];
        vs[2*i+1] = CMS.cornerOffsets[c2];
    }
    cubeLines.Vertices = vs;
    cubeLines.Transform.Pos = offset1;
    cubeLines.Color = Color.WHITE;
    var cubeLines2 = world.Clone(cubeLines);
    cubeLines2.Transform.Pos = offset2;
    world.CreateInstantly(cubeLines);
    world.CreateInstantly(cubeLines2);

    await Time.WaitSeconds(1.0f);
    world.EndCapture();

    (Vector3 v, Vector3 n)[] getIntersections(Mesh imesh, Vector3 o) {
        List<(Vector3 v, Vector3 n)> intersections = new();
        for (int i = 0; i < imesh.Indices.Length; i+=3) {
            Vector3 p1, p2, p3;
            p1 = imesh.Transform.Rot * imesh.Vertices[imesh.Indices[i]] + imesh.Transform.Pos;
            p2 = imesh.Transform.Rot * imesh.Vertices[imesh.Indices[i+1]] + imesh.Transform.Pos;
            p3 = imesh.Transform.Rot * imesh.Vertices[imesh.Indices[i+2]] + imesh.Transform.Pos;

            for (int e = 0; e < 12; e++) {
                var start = CMS.cornerOffsets[CMS.cubeEdgeCorners[e].Item1] + imesh.Transform.Pos + o;
                var end = CMS.cornerOffsets[CMS.cubeEdgeCorners[e].Item2] + imesh.Transform.Pos + o;

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
    var intersections2 = getIntersections(exampleMesh2, Vector3.ZERO);

    var allIntersections = new List<(Vector3 v, Vector3 n)>();
    allIntersections.AddRange(intersections11);
    allIntersections.AddRange(intersections12);
    allIntersections.AddRange(intersections2);
    var normalLines = new Line3D();
    normalLines.Color = Color.GREEN;
    normalLines.Width = 3.0f;
    normalLines.Vertices = allIntersections.SelectMany(i => new Vector3[] { i.v, i.v + 0.3f*i.n }).ToArray();
    world.CreateInstantly(normalLines);

    foreach (var i in allIntersections) {
        var s = new Sphere();
        s.Radius = 0.02f;
        s.Color = Color.YELLOW;
        s.Transform.Pos = i.v;
        world.CreateInstantly(s);
    }

    await orbitTask;
}

async Task AnimateAmbiguous2D() {
    var allEntities = world.BeginCapture();

    var gridPath1 = new PathBuilder();
    gridPath1.Grid(100.0f, new Vector2(-700.0f, -300.0f), new Vector2(-100.0f, 300.0f));
    var gridPath2 = new PathBuilder();
    gridPath2.Grid(100.0f, new Vector2(100.0f, -300.0f), new Vector2(700.0f, 300.0f));

    var gridShape1 = new Shape(gridPath1);
    gridShape1.ContourColor = Color.WHITE;
    world.CreateInstantly(gridShape1);
    var gridShape2 = new Shape(gridPath2);
    gridShape2.ContourColor = Color.WHITE;
    world.CreateInstantly(gridShape2);

    var examplePath = new PathBuilder();
    examplePath.MoveTo(new Vector2(-300.0f, 120.0f));
    examplePath.LineTo(new Vector2(-80.0f, -100.0f));
    examplePath.LineTo(new Vector2(-40.0f, 30.0f));
    examplePath.LineTo(new Vector2(120.0f, 30.0f));
    examplePath.LineTo(new Vector2(-40.0f, 150.0f));
    examplePath.LineTo(new Vector2(-40.0f, 70.0f));
    examplePath.Close();
    var exampleShape = new Shape(examplePath);
    exampleShape.Mode = ShapeMode.FilledContour;
    exampleShape.ContourColor = Color.BLACK;
    exampleShape.Color = new Color(1.0f, 0.1f, 0.1f, 0.5f);
    exampleShape.Transform.Pos = new Vector3(-300.0f, 0.0f, 0.0f);
    world.CreateInstantly(exampleShape);

    var examplePath2a = new PathBuilder();
    examplePath2a.MoveTo(new Vector2(-300.0f, 120.0f));
    examplePath2a.LineTo(new Vector2(-80.0f, -100.0f));
    examplePath2a.LineTo(new Vector2(-60.0f, 30.0f));
    examplePath2a.Close();

    var examplePath2b = new PathBuilder();
    examplePath2b.MoveTo(new Vector2(120.0f, 90.0f));
    examplePath2b.LineTo(new Vector2(-40.0f, 150.0f));
    examplePath2b.LineTo(new Vector2(-40.0f, 50.0f));
    examplePath2b.Close();

    var exampleShape2a = new Shape(examplePath2a);
    exampleShape2a.Mode = ShapeMode.FilledContour;
    exampleShape2a.ContourColor = Color.BLACK;
    exampleShape2a.Color = new Color(1.0f, 0.1f, 0.1f, 0.5f);
    exampleShape2a.Transform.Pos = new Vector3(500.0f, 0.0f, 0.0f);
    world.CreateInstantly(exampleShape2a);
    var exampleShape2b = new Shape(examplePath2b);
    exampleShape2b.Mode = ShapeMode.FilledContour;
    exampleShape2b.ContourColor = Color.BLACK;
    exampleShape2b.Color = new Color(1.0f, 0.1f, 0.1f, 0.5f);
    exampleShape2b.Path = examplePath2b;
    exampleShape2b.Transform.Pos = new Vector3(500.0f, 0.0f, 0.0f);
    world.CreateInstantly(exampleShape2b);

    // yellow highlight

    var highlightPath1 = new PathBuilder();
    highlightPath1.Rectangle(new Vector2(-400.0f, 0.0f), new Vector2(-300.0f, 100.0f));
    var highlightPath2 = new PathBuilder();
    highlightPath2.Rectangle(new Vector2(400.0f, 0.0f), new Vector2(500.0f, 100.0f));
    var highlightShape1 = new Shape(highlightPath1);  
    highlightShape1.Mode = ShapeMode.Contour;
    highlightShape1.ContourColor = 1.2f*Color.VIOLET;
    highlightShape1.SortKey = 1;
    highlightShape1.ContourSize = 3.0f;
    world.CreateInstantly(highlightShape1);
    var highlightShape2 = new Shape(highlightPath2);  
    highlightShape2.Mode = ShapeMode.Contour;
    highlightShape2.ContourColor = 1.2f*Color.VIOLET;
    highlightShape2.SortKey = 1;
    highlightShape2.ContourSize = 3.0f;
    world.CreateInstantly(highlightShape2);


    Vector2[] quadCorners1 = new Vector2[] {
        new Vector2(-400.0f, 0.0f),
        new Vector2(-300.0f, 0.0f),
        new Vector2(-300.0f, 100.0f),
        new Vector2(-400.0f, 100.0f),
    };

    Vector2[] quadCorners2 = new Vector2[] {
        new Vector2(400.0f, 0.0f),
        new Vector2(500.0f, 0.0f),
        new Vector2(500.0f, 100.0f),
        new Vector2(400.0f, 100.0f),
    };

    await Time.WaitSeconds(2.0f);

    bool[] cornerValues = new bool[] { true, false, true, false };
    Circle[] cornerCircles1 = new Circle[4];
    Circle[] cornerCircles2 = new Circle[4];
    for (int i = 0; i < 4; i++) {
        var c1 = new Circle(10.0f);
        c1.Color = cornerValues[i] ? Color.WHITE : Color.BLACK;
        c1.Transform.Pos = quadCorners1[i];
        c1.SortKey = 2;
        _ = world.CreateFadeIn(c1, 0.5f);
        cornerCircles1[i] = c1;
        var c2 = new Circle(10.0f);
        c2.Color = cornerValues[i] ? Color.WHITE : Color.BLACK;
        c2.Transform.Pos = quadCorners2[i];
        c2.SortKey = 2;
        await world.CreateFadeIn(c2, 0.5f);
        cornerCircles2[i] = c2;
    }

    await Time.WaitSeconds(1.0f);

    var caseText1 = new Text2D("1010");
    caseText1.Transform.Pos = new Vector3(-500.0f, -400.0f, 0.0f);
    caseText1.Size = 64.0f;
    var shapes = caseText1.CurrentShapes.Select(x => x.s).ToArray();
    shapes[0].Color = Color.WHITE;
    shapes[1].Color = Color.BLACK;
    shapes[2].Color = Color.WHITE;
    shapes[3].Color = Color.BLACK;
    var caseText2 = world.Clone(caseText1);
    caseText2.Transform.Pos = new Vector3(300.0f, -400.0f, 0.0f);
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
            task =  Animate.Move(mcs1[i].Transform, caseText1.Transform.Pos+shapes[i].Transform.Pos);
            task =  Animate.Move(mcs2[i].Transform, caseText2.Transform.Pos+shapes[i].Transform.Pos);
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
        pb.Rectangle(new Vector2(0.0f), new Vector2(100.0f));
        var shape = new Shape(pb);
        shape.Mode = ShapeMode.Contour;
        shape.ContourColor = Color.WHITE;
        shape.ContourSize = 3.0f;
        shape.Transform.Pos = new Vector3(pos, 0.0f);
        shape.SortKey = 1;
        _ = world.CreateFadeIn(shape, 0.5f);
        for (int i = 0; i < edges.Length; i+=2) {
            var (c11, c12) = CMS.quadEdgeCorners[edges[i]];
            var (c21, c22) = CMS.quadEdgeCorners[edges[i+1]];
            var pb2 = new PathBuilder();
            var repCheck = new int[] { c11, c12, c21, c22 };
            // find repeated corner
            if (!invert) {
                pb2.MoveTo(50f*(CMS.quadCorners[c11] + CMS.quadCorners[c12]));
                pb2.LineTo(50f*(CMS.quadCorners[c21] + CMS.quadCorners[c22]));
                int rep = -1;
                for (int j = 0; j < 4; j++) {
                    if (repCheck.Count(x => x == repCheck[j]) > 1) {
                        rep = repCheck[j];
                        break;
                    }
                }
                pb2.LineTo(100f*CMS.quadCorners[rep]);
            } else {
                pb2.MoveTo(100.0f*new Vector2(0.0f));
                pb2.LineTo(100.0f*new Vector2(0.5f, 0.0f));
                pb2.LineTo(100.0f*new Vector2(1.0f, 0.5f));
                pb2.LineTo(100.0f*new Vector2(1.0f, 1.0f));
                pb2.LineTo(100.0f*new Vector2(0.5f, 1.0f));
                pb2.LineTo(100.0f*new Vector2(0.0f, 0.5f));
            }
            var shape2 = new Shape(pb2);
            shape2.Mode = ShapeMode.FilledContour;
            shape2.ContourColor = Color.WHITE;
            shape2.ContourSize = 3.0f;
            shape2.Transform.Pos = new Vector3(pos, 0.0f);
            _ = world.CreateFadeIn(shape2, 0.5f);
        }
        var labelShape = new Text2D(label);
        labelShape.Size = 24.0f;
        labelShape.Color = Color.WHITE;
        labelShape.Transform.Pos = pos + new Vector2(-20.0f, 120.0f);
        _ = world.CreateFadeIn(labelShape, 0.5f);
        await Time.WaitSeconds(0.5f);
    }

    await showCase(new int[] { 1, 0, 3, 2}, new Vector2(-100.0f, 350.0f), "10a (1010)");
    await showCase(new int[] { 3, 0, 1, 2}, new Vector2(100.0f, 350.0f), "10b (1010)", invert: true);


    await Time.WaitSeconds(1.0f);

    async Task moveGrid2() {
        float moveAmount = 2000.0f;
        var task = Animate.Offset(gridShape2.Transform, new Vector3(moveAmount, 0.0f, 0.0f));
        foreach(var shape in shapes2) {
            task = Animate.Offset(shape.Transform, new Vector3(moveAmount, 0.0f, 0.0f));
        }
        _ = Animate.Offset(exampleShape2a.Transform, new Vector3(moveAmount, 0.0f, 0.0f));
        _ = Animate.Offset(exampleShape2b.Transform, new Vector3(moveAmount, 0.0f, 0.0f));
        foreach(var cc in cornerCircles2) {
            _ = Animate.Offset(cc.Transform, new Vector3(moveAmount, 0.0f, 0.0f));
        }
        _ = Animate.Offset(highlightShape2.Transform, new Vector3(moveAmount, 0.0f, 0.0f));
        for (int i = 0; i < 4; i ++) {
            _ = Animate.Offset(mcs2Tasks[i].Result.Transform, new Vector3(moveAmount, 0.0f, 0.0f));
        }

        moveAmount = 450.0f;
        task = Animate.Offset(gridShape1.Transform, new Vector3(moveAmount, 0.0f, 0.0f));
        foreach(var shape in shapes) {
            task = Animate.Offset(shape.Transform, new Vector3(moveAmount, 0.0f, 0.0f));
        }
        _ = Animate.Offset(exampleShape.Transform, new Vector3(moveAmount, 0.0f, 0.0f));
        foreach(var cc in cornerCircles1) {
            _ = Animate.Offset(cc.Transform, new Vector3(moveAmount, 0.0f, 0.0f));
        }
        _ = Animate.Offset(highlightShape1.Transform, new Vector3(moveAmount, 0.0f, 0.0f));
        for (int i = 0; i < 4; i ++) {
            _ = Animate.Offset(mcs1Tasks[i].Result.Transform, new Vector3(moveAmount, 0.0f, 0.0f));
        }


        await task;
    }
    await moveGrid2();


    var gridBase = new Vector2(50.0f, 0.0f);

    (Vector2, Vector2, bool)[] intersections1 = new (Vector2, Vector2, bool)[] {
        (new Vector2(50.0f, 0.0f), new Vector2(1.0f, -0.3f).Normalized, true),
        (new Vector2(100.0f, 33.0f), new Vector2(0.0f, -1.0f).Normalized, false),
        (new Vector2(0.0f, 80.0f), new Vector2(0.15f, 1.0f).Normalized, false),
        (new Vector2(60.0f, 100.0f), new Vector2(-1.0f, 0.0f).Normalized, true),
    };

    var caseLinePath = new PathBuilder();
    caseLinePath.MoveTo(new Vector2(-20.0f, 0.0f));
    caseLinePath.LineTo(new Vector2(120.0f, 0.0f));
    var caseLine = new Shape(caseLinePath);
    caseLine.Mode = ShapeMode.Contour;
    caseLine.ContourColor = 1.3f*Color.YELLOW;
    caseLine.ContourSize = 3.0f;
    caseLine.Transform.Pos = new Vector2(-100.0f, 330.0f);
    caseLine.SortKey = 1;
    world.CreateInstantly(caseLine);
    await Animate.InterpF(x => {
        caseLine.ContourColor = Color.Lerp(Color.TRANSPARENT, 1.3f*Color.YELLOW, x);
    }, 0.0f, 1.0f, 0.5f);

    List<Shape> intersectionShapes = new List<Shape>();
    foreach (var intr in intersections1) {
        var c = new Circle(8.0f);
        c.Color = Color.YELLOW;
        c.Transform.Pos = intr.Item1 + gridBase;
        c.SortKey = 2;
        var pb = new PathBuilder();
        pb.MoveTo(intr.Item1);
        pb.LineTo(intr.Item1 + 30.0f*intr.Item2);
        var l = new Shape(pb);
        l.Mode = ShapeMode.Contour;
        l.ContourColor = Color.GREEN;
        l.Transform.Pos = gridBase;
        l.SortKey = 3;
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
                float len = 100.0f;
                tpb.LineTo(intersections1[i].Item1 
                    + (!intersections1[i].Item3 ? len*intersections1[i].Item2.PerpCw : len*intersections1[i].Item2.PerpCcw));
                var tl = new Shape(tpb);
                tl.Mode = ShapeMode.Contour;
                tl.ContourColor = Color.MAGENTA;
                tl.Transform.Pos = gridBase;
                tl.SortKey = 4;
                tl.ContourSize = 2.0f;
                task = world.CreateFadeIn(tl, 0.5f);
                nws.Add(tl);
            }
            var intr = new Circle(8.0f);
            intr.Color = Color.ORANGE;
            intr.Transform.Pos = gridBase + v*100.0f;
            intr.SortKey = 2;
            task = world.CreateFadeIn(intr, 0.5f);
            garbage.Add(intr);
            await task;
            await Time.WaitSeconds(0.5f);
            foreach (var s in nws) {
                world.Destroy(s);
            }
            var pb = new PathBuilder();
            pb.MoveTo(intersections1[p1].Item1);
            pb.LineTo(v*100.0f);
            pb.LineTo(intersections1[p2].Item1);
            var l = new Shape(pb);
            l.Mode = ShapeMode.Contour;
            l.ContourColor = Color.ORANGE;
            l.Transform.Pos = gridBase;
            l.SortKey = 3;
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

    await Animate.Offset(caseLine.Transform, new Vector3(200.0f, 0.0f, 0.0f));
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
        if (!s.managedLifetime) {
            world.Destroy(s);
        }
    }
}

async Task<Line3D> AnimateGrid(Vector3 min, float step, int size, Color color) {
    var showCube = new Cube();
    showCube.Color = Color.WHITE;
    showCube.Transform.Scale = new Vector3(3.0f);
    world.CreateInstantly(showCube);
    await Animate.InterpF(x => {
        showCube.Color = Color.Lerp(Color.TRANSPARENT, Color.WHITE, x);
        showCube.Outline = Color.Lerp(Color.TRANSPARENT, Color.BLACK, x);
    }, 0.0f, 1.0f, 1.0f);

    await Time.WaitSeconds(2.0f); 

    var grid = new Line3D(mode: MeshVertexMode.Segments);
    grid.Color = color;
    grid.Width = 2.0f;
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
        var sphere = new Sphere();
        sphere.Radius = 0.05f;
        sphere.Color = Color.ORANGE;
        sphere.Transform.Pos = min + new Vector3(i*step, j*step, k*step);
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
        var sample = tinyCube(s.Transform.Pos);
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
        var s = new Sphere();
        s.Transform.Pos = p;
        s.Radius = 0.03f;
        s.Color = Color.YELLOW;
        sphereList.Add(s);
        world.CreateInstantly(s);
        await Time.WaitSeconds(0.01f);
    }
    await Time.WaitSeconds(1.0f);

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
    normalLines.Width = 4.0f;
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
        q.Width = 2.1f;

        quads[i] = q;
        q.Transform.Pos = quadOffset + CMS.quadOffsets[i];
        _ = world.CreateFadeIn(q, 0.5f);

        for (int j = 0; j < 4; j++) {
            var s = new Sphere();
            s.Radius = 0.051f;
            s.Transform.parent = q.Transform;
            s.Transform.Pos = new Vector3(faceQuadPositions[i, j], 0.0f);
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

    quads[5].Transform.parent = quads[1].Transform;
    quads[5].Transform.Pos = new Vector3(1.0f, 0.0f, 0.0f);
    Vector3[] ps = quads.Select(q => q.Transform.Pos).ToArray();
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
        l.Width = 4.0f;
        issN[i] = l;
    }

    (Transform, Vector3, Vector3, bool)[] iPts = new (Transform, Vector3, Vector3, bool)[8] {
        (quads[0].Transform, new Vector3(0.5f, 0.0f, 0.0f), new Vector3(-1.0f, 0.0f, 0.0f), false),
        (quads[5].Transform, new Vector3(0.5f, 0.0f, 0.0f), new Vector3( 1.0f, 0.0f, 0.0f), true),
        (quads[1].Transform, new Vector3(1.0f, 0.5f, 0.0f), new Vector3( 0.0f, 1.0f, 0.0f), true),
        (quads[1].Transform, new Vector3(0.0f, 0.5f, 0.0f), new Vector3( 0.0f, 1.0f, 0.0f), false),
        (quads[4].Transform, new Vector3(0.5f, 0.0f, 0.0f), new Vector3(-1.0f, 0.0f, 0.0f), false),
        (quads[0].Transform, new Vector3(1.0f, 0.5f, 0.0f), new Vector3( 0.0f, 1.0f, 0.0f), true),
        (quads[4].Transform, new Vector3(0.5f, 1.0f, 0.0f), new Vector3(-1.0f, 0.0f, 0.0f), true),
        (quads[5].Transform, new Vector3(0.0f, 0.5f, 0.0f), new Vector3( 0.0f, 1.0f, 0.0f), false),
    };

    for (int i = 0; i < 8; i++) {
        iss[i].Transform.parent = iPts[i].Item1;
        iss[i].Transform.Pos = iPts[i].Item2;
        issN[i].Transform.parent = iPts[i].Item1;
        issN[i].Transform.Pos = iPts[i].Item2;
        issN[i].Vertices = new Vector3[] { Vector3.ZERO, 0.3f*iPts[i].Item3 };
        _ = world.CreateInstantly(iss[i]);
        _ = world.CreateInstantly(issN[i]);
    }

    var cam = world.ActiveCamera;
    var camOrigRot = cam.Transform.Rot;
    var camOrigPos = cam.Transform.Pos;

    var targetPosition = cam.Transform.Pos + new Vector3(-4.0f, 0.0f, 9.0f);
    var targetLookAt = min + new Vector3(0.5f, 3.5f, 1.5f);
    var toTarget = targetLookAt - targetPosition;
    var targetRot = Quaternion.LookRotation(toTarget, Vector3.UP);
    var startRot = cam.Transform.Rot;
    var startPos = cam.Transform.Pos;
    await Animate.InterpF(x => {
        cam.Transform.Pos = Vector3.Lerp(startPos, targetPosition, x);
        cam.Transform.Rot = Quaternion.Slerp(startRot, targetRot, x);
    }, 0.0f, 1.0f, 1.0f);

    await hideGrid(true);

    await FoldAll(quadOffset, ps, quads, 1.0f, MathF.PI/2.0f, 0.0f);

    // separete quads
    foreach (var q in quads) {
        if (q == quads[0]) continue;
        var d = q.Transform.Pos - quads[0].Transform.Pos;
        if (q == quads[5]) d = Vector3.RIGHT;
        d.z = 0.0f;
        d = d.Normalized;
        _ = Animate.Offset(q.Transform, d * 0.5f);
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
        var s = new Sphere();
        s.Radius = 0.03f;
        s.Color = Color.ORANGE;
        s.Transform.Pos = new Vector3(0.5f, 0.5f, 0.0f);
        s.Transform.parent = quads[i].Transform;
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
            l.Width = 4.0f;
            l.Transform.parent = quads[i].Transform;
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
    foreach (var q in quads) {
        if (q == quads[0]) continue;
        var d = q.Transform.Pos - quads[0].Transform.Pos;
        if (q == quads[5]) d = Vector3.RIGHT;
        d.z = 0.0f;
        d = d.Normalized;
        _ = Animate.Offset(q.Transform, -d * 0.5f);
    }
    await Time.WaitSeconds(1.0f);
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
            cam.Transform.Pos = Vector3.Lerp(startPos, targetPosition, x);
            cam.Transform.Rot = Quaternion.Slerp(startRot, targetRot, x);
        }, 1.0f, 0.0f, 1.0f);
        await Animate.OrbitAndLookAt(cam.Transform, Vector3.UP, Vector3.ZERO, 720.0f, 20.0f);
    }
    var camMoveTask = moveCam();
    await hideGrid(false);

    cmsCubeLines = await CubicalMarchingSquares2(positions, step, tinyCube, tinyCubeNormal, stepWait: 0.1f, cellVertices: cellVertices);

    var cornerVerts = cellVertices[0, 3, 0].ToArray();
    var cornerLines = new Line3D();
    cornerLines.Color = Color.YELLOW;
    cornerLines.Vertices = cornerVerts;
    cornerLines.Width = 4.0f;
    world.CreateInstantly(cornerLines);

    await camMoveTask;

    await Animate.InterpF(x => {
        cmsCubeLines.Color = Color.Lerp(cmsCubeLines.Color, Color.TRANSPARENT, x);
    }, 0.0f, 1.0f, 1.0f);
    world.Destroy(cmsCubeLines);


    targetPosition = cam.Transform.Pos + new Vector3(-4.0f, 0.0f, 9.0f);
    targetLookAt = min + new Vector3(0.5f, 3.5f, 0.5f);
    toTarget = targetLookAt - targetPosition;
    targetRot = Quaternion.LookRotation(toTarget, Vector3.UP);

    //_ = hideGrid(true);

    (Vector3, Vector3)[] cornerNormals = new (Vector3, Vector3)[3] {
        (new Vector3(1.0f, 3.0f, 0.5f), new Vector3(1.0f, 3.0f, 0.2f)),
        (new Vector3(0.5f, 3.0f, 1.0f), new Vector3(0.2f, 3.0f, 1.0f)),
        (new Vector3(1.0f, 3.5f, 1.0f), new Vector3(1.0f, 3.8f, 1.0f)),
    };

    // move camera back
    await Animate.InterpF(x => {
        cam.Transform.Pos = Vector3.Lerp(startPos, targetPosition, x);
        cam.Transform.Rot = Quaternion.Slerp(startRot, targetRot, x);
    }, 0.0f, 1.0f, 1.0f);

    var cornerLines2 = new Line3D[3];
    for (int i = 0; i < 3; i++) {
        var cornerLine = new Line3D();
        cornerLine.Color = Color.GREEN;
        cornerLine.Vertices = new Vector3[] { cornerNormals[i].Item1 + min + new Vector3(0.001f), cornerNormals[i].Item2 + min + new Vector3(0.001f) };
        cornerLine.Width = 4.0f;
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
        q.Vertices = (p1 + min, p2 + min, p3 + min, p4 + min);
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

    var solved = new Sphere();
    solved.Radius = 0.03f;
    solved.Color = 1.5f*Color.RED;
    solved.Transform.Pos = new Vector3(0.5f, 3.5f, 0.5f) + min;
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

    var mesh = new Mesh();
    List<Vector3> fanV = new ();
    List<uint> fanI = new ();
    // create fan to solved.Transform.Pos
    for (int i = 0; i < 6; i++) {
        fanV.Add(fanVertices[i] + min);
    }
    fanV.Add(solved.Transform.Pos);
    for (int i = 0; i < 6; i++) {
        fanI.Add((uint)i);
        fanI.Add(6);
        fanI.Add((uint)(i+1)%6);
    }
    mesh.Vertices = fanV.ToArray();
    mesh.Indices = fanI.ToArray();
    mesh.Color = Color.ORANGE;
    await world.CreateFadeIn(mesh, 0.5f);

    await Time.WaitSeconds(1.0f);

    targetLookAt = min + new Vector3(0.5f, 2.5f, 1.5f);
    toTarget = targetLookAt - targetPosition;
    targetRot = Quaternion.LookRotation(toTarget, Vector3.UP);
    startRot = cam.Transform.Rot;
    await Animate.InterpF(x => {
        cam.Transform.Rot = Quaternion.Slerp(startRot, targetRot, x);
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
        cornerLine.Width = 4.0f;
        _ = world.CreateFadeIn(cornerLine, 0.5f);
        cornerLines2[i] = cornerLine;
    }
    Line3D cornerContour = new Line3D();
    cornerContour.Color = Color.YELLOW;
    cornerContour.Width = 4.0f;
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
    var centerSphere = new Sphere();
    centerSphere.Radius = 0.03f;
    centerSphere.Color = 1.5f*Color.RED;
    centerSphere.Transform.Pos = centerVert + min;
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
    fanMesh2.Color = Color.ORANGE;
    fanMesh2.Outline = Color.BLACK;
    await world.CreateFadeIn(fanMesh2, 0.5f);

    await Time.WaitSeconds(1.0f);

    targetLookAt = min + new Vector3(0.5f, 3.5f, 1.5f);
    toTarget = targetLookAt - targetPosition;
    targetRot = Quaternion.LookRotation(toTarget, Vector3.UP);
    startRot = cam.Transform.Rot;

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
        cam.Transform.Rot = Quaternion.Slerp(startRot, targetRot, x);
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
        cornerLine.Width = 4.0f;
        _ = world.CreateFadeIn(cornerLine, 0.5f);
        cornerLines2[i] = cornerLine;
    }

    var cornerContour2 = new Line3D();
    cornerContour2.Color = Color.YELLOW;
    cornerContour2.Width = 4.0f;
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

    var sharpSp1 = new Sphere();
    sharpSp1.Radius = 0.03f;
    sharpSp1.Color = 1.5f*Color.VIOLET;
    sharpSp1.Transform.Pos = new Vector3(0.5f, 3.5f, 1.0f) + min;
    var sharpSp2 = world.Clone(sharpSp1);
    sharpSp2.Transform.Pos = new Vector3(0.5f, 3.5f, 2.0f) + min;
    _ = world.CreateFadeIn(sharpSp1, 0.5f);
    await world.CreateFadeIn(sharpSp2, 0.5f);
    _ = Animate.Color(sharpSp1, Color.VIOLET, 0.5f);
    await Animate.Color(sharpSp2, Color.VIOLET, 0.5f);
    sharpSphereList.Add(sharpSp1);
    sharpSphereList.Add(sharpSp2);
    
    await Time.WaitSeconds(1.0f);

    var blueLine = new Line3D();
    blueLine.Color = Color.BLUE;
    blueLine.Width = 4.0f;
    blueLine.Vertices = new Vector3[] {
        new Vector3(0.5f, 3.5f, 1.0f) + min + new Vector3(0.002f),
        new Vector3(0.5f, 3.5f, 2.0f) + min + new Vector3(0.002f),
    };
    _ = world.CreateFadeIn(blueLine, 0.5f);
    await Time.WaitSeconds(1.0f);
    await Animate.Color(blueLine, Color.YELLOW, 0.5f);

    var sharpCenter1 = new Sphere();
    sharpCenter1.Radius = 0.03f;
    sharpCenter1.Color = 1.5f*Color.RED;
    sharpCenter1.Transform.Pos = new Vector3(0.75f, 3.5f, 1.5f) + min;
    var sharpCenter2 = world.Clone(sharpCenter1);
    sharpCenter2.Transform.Pos = new Vector3(0.5f, 3.25f, 1.5f) + min;
    _ = world.CreateFadeIn(sharpCenter1, 0.5f);
    await world.CreateFadeIn(sharpCenter2, 0.5f);
    _ = Animate.Color(sharpCenter1, Color.RED, 0.5f);
    await Animate.Color(sharpCenter2, Color.RED, 0.5f);
    sharpSphereList.Add(sharpCenter1);
    sharpSphereList.Add(sharpCenter2);

    await Time.WaitSeconds(1.0f);

    Mesh fanMesh31 = new Mesh();
    fanV.Clear();
    fanI.Clear();
    fanV.Add(new Vector3(0.5f, 3.0f, 1.0f) + min);
    fanV.Add(new Vector3(0.5f, 3.0f, 2.0f) + min);
    fanV.Add(new Vector3(0.5f, 3.5f, 2.0f) + min);
    fanV.Add(new Vector3(0.5f, 3.5f, 1.0f) + min);
    fanV.Add(sharpCenter2.Transform.Pos);
    fanI.AddRange([0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4]);
    fanMesh31.Vertices = fanV.ToArray();
    fanMesh31.Indices = fanI.ToArray();
    fanMesh31.Color = Color.ORANGE;
    fanMesh31.Outline = Color.BLACK;
    await world.CreateFadeIn(fanMesh31, 0.5f);

    await Time.WaitSeconds(1.0f);

    var fanMesh32 = world.Clone(fanMesh31);
    fanV.Clear();
    fanI.Clear();
    fanV.Add(new Vector3(0.5f, 3.5f, 1.0f) + min);
    fanV.Add(new Vector3(0.5f, 3.5f, 2.0f) + min);
    fanV.Add(new Vector3(1.0f, 3.5f, 2.0f) + min);
    fanV.Add(new Vector3(1.0f, 3.5f, 1.0f) + min);
    fanV.Add(sharpCenter1.Transform.Pos);
    fanI.AddRange([0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4]);
    fanMesh32.Vertices = fanV.ToArray();
    fanMesh32.Indices = fanI.ToArray();
    fanMesh32.Color = Color.ORANGE;
    fanMesh32.Outline = Color.BLACK;
    await world.CreateFadeIn(fanMesh32, 0.5f);

    await Time.WaitSeconds(1.0f);

    var wholeMesh = DoCMS2(tinyCube, tinyCubeNormal, nmax: 0);
    wholeMesh.Color = Color.ORANGE;
    wholeMesh.Outline = Color.BLACK;
    async Task createMesh() {
        await world.CreateFadeIn(wholeMesh, 0.5f);
        await Animate.InterpF(x => {
            foreach(var cl in cornerLines2) {
                cl.Color = Color.Lerp(cl.Color, Color.TRANSPARENT, x);
            }
            blueLine.Color = Color.Lerp(blueLine.Color, Color.TRANSPARENT, x);
            cornerLines.Color = Color.Lerp(cornerLines.Color, Color.TRANSPARENT, x);
            cornerContour.Color = Color.Lerp(cornerContour.Color, Color.TRANSPARENT, x);
            cornerContour2.Color = Color.Lerp(cornerContour2.Color, Color.TRANSPARENT, x);
            fanMesh31.Color = Color.Lerp(fanMesh31.Color, Color.TRANSPARENT, x);
            fanMesh31.Outline = Color.Lerp(fanMesh31.Outline, Color.TRANSPARENT, x);
            fanMesh32.Color = Color.Lerp(fanMesh32.Color, Color.TRANSPARENT, x);
            fanMesh32.Outline = Color.Lerp(fanMesh32.Outline, Color.TRANSPARENT, x);
            fanMesh2.Color = Color.Lerp(fanMesh2.Color, Color.TRANSPARENT, x);
            fanMesh2.Outline = Color.Lerp(fanMesh2.Outline, Color.TRANSPARENT, x);
            solved.Color = Color.Lerp(solved.Color, Color.TRANSPARENT, x);
            mesh.Color = Color.Lerp(mesh.Color, Color.TRANSPARENT, x);
            foreach (var s in sharpSphereList) {
                s.Color = Color.Lerp(s.Color, Color.TRANSPARENT, x);
            }
        }, 0.0f, 1.0f, 1.0f);
        world.Destroy(blueLine);
        world.Destroy(cornerLines);
        world.Destroy(cornerContour);
        world.Destroy(cornerContour2);
        world.Destroy(cornerLines2);
        world.Destroy(fanMesh31);
        world.Destroy(fanMesh32);
        world.Destroy(fanMesh2);
        world.Destroy(solved);
        world.Destroy(mesh);
        world.Destroy(sharpSphereList.ToArray());
    }
    _ = createMesh();

    startRot = cam.Transform.Rot;
    await Animate.InterpF(x => {
        cam.Transform.Rot = Quaternion.Slerp(startRot, camOrigRot, x);
        cam.Transform.Pos = Vector3.Lerp(targetPosition, camOrigPos, x);
    }, 0.0f, 1.0f, 1.0f);
    await Animate.OrbitAndLookAt(cam.Transform, Vector3.UP, Vector3.ZERO, 360.0f, 5.0f);

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

void Test() {
    var p = CMS.FacePosToCubePos(3, new Vector2(0.25f, 0.25f));
    //Debug.Assert(p == new Vector3(0.0f, 0.25f, 0.25f));
}
}
