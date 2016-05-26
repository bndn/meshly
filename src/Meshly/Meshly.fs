/// Copyright (C) 2016 The Authors.
module Meshly

open Kdtree
open Plyser
open Vector
open Point

type Normal = Flat | Smooth

type Vertex = Point * Vector * float * float

type Face = int * int * int

[<NoComparison>]
type Mesh =
    {
        Vertices: Vertex array;
        Normal: Normal;
        Faces: Face kdtree;
        Bounds: Point * float * float * float;
    }

[<NoComparison>]
type Intersection =
    {
        Distance: float;
        Normal: Vector;
        Coordinates: float * float;
    }

[<Literal>]
let Epsilon = 10e-12

/// <summary>
/// Get a coordinate from a PLY element.
/// </summary>
/// <param name=a>Axis to get the coordinate for.</param>
/// <param name=e>PLY element to get the coordinate for.</param>
/// <returns>The float value of a axis-coordinate of a PLY element.</returns>
let coord a e =
    match Plyser.getFloat a e with
    | Some c -> float c
    | _ -> failwith (sprintf "No %s-coordinate found for: %A" a e)

/// <summary>
/// Get a normal from a PLY element.
/// </summary>
/// <param name=a>Axis to get the normal for.</param>
/// <param name=e>PLY element to get the normal for.</param>
/// <returns>A normal option coordinate for an axis.</returns>
let norm a e =
    match Plyser.getFloat ("n" + a) e with
    | Some n -> Some(float n)
    | None   -> None

/// <summary>
/// Get U, V coordinates from a PLY element.
/// </summary>
/// <param name=e>PLY element to get the U, V coordinates for.</param>
/// <returns>A U, V coordinate for a PLY element (0., 0. if None).</returns>
let uv e =
    match (Plyser.getFloat "u" e, Plyser.getFloat "v" e) with
    | Some u, Some v -> float v, float u
    | _              -> 0., 0.

/// <summary>
/// Create a vertex from a PLY element.
/// </summary>
/// <param name=e>The PLY element to create a vertex for.</param>
/// <returns>A vertex for a mesh.</returns>
let vertex e =
    let p = Point.make (coord "x" e) (coord "y" e) (coord "z" e)

    let nx, ny, nz = norm "x" e, norm "y" e, norm "z" e
    let u, v = uv e

    let n = match nx, ny, nz with
            | Some nx, Some ny, Some nz -> Some (Vector.make nx ny nz)
            | _                         -> None

    p, n, u, v

/// <summary>
/// Create a face from a PLY element.
/// </summary>
/// <param name=e>The PLY element to create a face for.</param>
/// <returns>A face element, consisting of the IDs of 3 vertices.</returns>
let face e =
    match Plyser.getIntList "vertex_indices" e with
    | Some [a; b; c] -> a, b, c
    | _ -> match Plyser.getUIntList "vertex_indices" e with
           | Some [a; b; c] -> int a, int b, int c
           | _ -> failwith (sprintf "No vertices found for: %A" e)

/// <summary>
/// Compute the bounds of a face.
/// <summary>
/// <param name=vs>Vertices to create a bounds for.</param>
/// <param name=it>
/// Index triplet, containing the vertex index for the three
/// vertices of the triangle.
/// </param>
/// <returns>Bounds for the face.</returns>
let bounds vs (a, b, c) =
    let a, _, _, _ = Array.get vs a
    let b, _, _, _ = Array.get vs b
    let c, _, _, _ = Array.get vs c

    let ax, ay, az = Point.getCoord a
    let bx, by, bz = Point.getCoord b
    let cx, cy, cz = Point.getCoord c

    let lx, ly, lz =
        float (min ax (min bx cx)),
        float (min ay (min by cy)),
        float (min az (min bz cz))

    let hx, hy, hz =
        float (max ax (max bx cx)),
        float (max ay (max by cy)),
        float (max az (max bz cz))

    Point.make (lx - Epsilon) (ly - Epsilon) (lz - Epsilon),
    (hx - lx) + Epsilon * 2.,
    (hy - ly) + Epsilon * 2.,
    (hz - lz) + Epsilon * 2.

/// <summary>
/// Compute the intersection of a ray and a face, using Möller-Trumbore.
/// </summary>
/// <param name=r>Ray to check for intersection with.</param>
/// <param name=m>Mesh to find triangles in for intersection.</param>
/// <param name=it>
/// Index triplet, containing the vertex index for the three
/// vertices of the triangle.
/// </param>
/// <returns>An intersection, if any.</returns>
let intersect r m (a, b, c) =
    let d = Ray.getVector r
    let o = Ray.getOrigin r

    let a, an, ua, va = m.Vertices.[a]
    let b, bn, ub, vb = m.Vertices.[b]
    let c, cn, uc, vc = m.Vertices.[c]

    let e1 = Point.distance a b
    let e2 = Point.distance a c
    let p = Vector.crossProduct d e2
    let det = e1 * p

    if det > -Epsilon && det < Epsilon then None else

    let inv = 1. / det

    let t = Point.distance a o
    let u = (t * p) * inv

    if u < 0. || u > 1. then None else

    let q = Vector.crossProduct t e1
    let v = (d * q) * inv

    if v < 0. || u + v > 1. then None else

    let h = (e2 * q) * inv

    if h <= Epsilon then None else

    // Calculate alpha, beta, gamma values
    let beta = u
    let gamma = v
    let alpha = 1. - beta - gamma

    // Get uv-texture mapping values
    let ut = alpha * ua + beta * ub + gamma * uc
    let vt = alpha * va + beta * vb + gamma * vc

    let n = match m.Normal with
            | Flat   -> let c = Vector.crossProduct e1 e2
                        (1. / (Vector.magnitude c)) * c
            | Smooth -> Vector.normalise
                        <| alpha * an + beta * bn + gamma * cn

    let n = if d * n > 0. then -n else n

    Some(h, n, ut, vt)

/// <summary>
/// Generate components necessary to do smooth shading from a parsed
/// PLY-file.
/// </summary>
/// <param name=p>The parsed PLY file.</param>
/// <returns>
/// A quadruplet containing a list of vertices (ordered from index 0-N),
/// a map of vertices with their ID as key, a list of faces and a map
/// of faces with the vertices that the face connects to as keys.
/// "yuck!" - Andreas (who built it)
/// </returns>
let makeSmooth p =
    let appendKvp (k, v) m =
        m |> Map.add k (if Map.containsKey k m
                        then v :: (Map.find k m)
                        else [v])

    let vs, vm, fs, fm, _ =
        p |> List.fold (fun (vs, vm, fs, fm, i) e ->
            match Plyser.getName e with
            | "vertex" -> let v = vertex e
                          let vm = Map.add i (ref v) vm
                          (v :: vs, vm, fs, fm, i + 1)
            | "face"   -> let f = face e
                          let a, b, c = f
                          let fm = fm |> appendKvp (a, ref f)
                                      |> appendKvp (b, ref f)
                                      |> appendKvp (c, ref f)
                          (vs, vm, f :: fs, fm, i)
            | _        -> (vs, vm, fs, fm, i)
        ) ([], Map.empty, [], Map.empty, 0)

    List.rev vs, vm, fs, fm

/// <summary>
/// Generate components necessary to render a flat shaded Mesh from
/// a parsed PLY file.
/// </summary
/// <param name=p>The parsed PLY file.</param>
/// <returns>A list of vertices and a list of faces as a tuple.</returns>
let makeFlat p =
    // foldback to return the list of vertices in the order from 0-N,
    // avoids reversing the list after folding
    List.foldBack (fun e (vs, fs) ->
        match Plyser.getName e with
        // For triangle meshes we're only interested in "vertex" and "face"
        // elements; disassemble these when encountered.
        | "vertex" -> let p, n, u, v = vertex e
                      let n = if n.IsSome then n.Value else Vector.make 0. 0. 0.
                      let v = p, n, u, v   in (v :: vs, fs)
        | "face"   -> let f = face e       in (vs, f :: fs)
        // Ignore any elements that aren't part of triangle meshes.
        | _ -> (vs, fs)
    ) p ([], [])

/// <summary>
/// Given a list of vertices, a map of vertices and a map of faces,
/// generates normals for the vertices unless it is already specified
/// in the PLY file.
/// </summary>
/// <remarks>
/// <para>The normal of a vertex is defined as the normalised sumvector of
/// normals on the faces surrounding the vertex.</para>
/// <para>Maps are used to do lookups in, taking O(log N) for find.</para>
/// </remarks>
/// <param name=vs>Vertex list to run through (ordered from 0-N).</param>
/// <param name=vm>
/// Vertex map, with the vertex ID as key and a reference to the vertex
/// as value.
/// </param>
/// <param name=fm>
/// Faces map, having vertex IDs as keys, and a list of face references for
/// each face connected to that vertex.
/// </param>
/// <returns>A list of vertices with normals.</returns>
let generateSmoothNormals vs vm fm =
    vs |> List.mapi (fun i v ->
        let p, (n:Vector option), tu, tv = v
        let zv = Vector.make 0. 0. 0.
        // if the ply already has vertex normals stored
        if n.IsSome then (p, n.Value, tu, tv) else
        if not (Map.containsKey i fm) then (p, zv, tu, tv) else
        let normals =
            Map.find i fm |> List.map (fun f ->
                let a, b, c = !f
                let a, _, _, _ = !(Map.find a vm)
                let b, _, _, _ = !(Map.find b vm)
                let c, _, _, _ = !(Map.find c vm)
                let u = Point.distance a b
                let v = Point.distance a c
                Vector.normalise (Vector.crossProduct u v)
            )

        let n = normals
                |> List.reduce (fun ns n -> ns + n)
                |> fun n ->
                    if n = zv then zv else Vector.normalise n
        p, n, tu, tv
    )

/// <summary>
/// Create a triangle mesh from a PLY-formatted string.
/// </summary>
/// <param name=s>The input string.</param>
/// <param name=n>Define if the normals are flat- or smooth shaded.</param>
/// <returns>The created mesh.</returns>
let make s n =
    // First, parse the input string,
    let parsed = Plyser.parse s

    // .. and then disassemble the PLY elements to a list of vertices
    // and faces. The vertices get their normals calculated (if they
    // are not included in the PLY-file) for smooth shading.
    let vs, fs =
        match n with
        | Flat   -> makeFlat parsed
        | Smooth -> let vs, vm, fs, fm = makeSmooth parsed
                    (generateSmoothNormals vs vm fm), fs

    // Convert the vertice list to an array to allow random
    // access of elements.
    let vs = List.toArray vs

    // Compute the bounds of all the faces so they can be inserted into a
    // kd-tree.
    let fs = fs |> ([] |> List.fold (fun fs f -> (f, bounds vs f) :: fs))

    let f (xv, xa, yv, ya, zv, za) (_, (p, w, h, d)) =
        let x, y, z = Point.getCoord p

        min xv x, max xa x + w,
        min yv y, max ya y + h,
        min zv z, max za z + d

    let i = infinity

    let xv, xa, yv, ya, zv, za = fs |> List.fold f (i, -i, i, -i, i, -i)

    {
        Vertices = vs;
        Normal = n;
        Faces = Kdtree.make fs;
        Bounds = Point.make xv yv zv, xa - xv, ya - yv, za - zv
    }

/// <summary>
/// Intersect a ray with a triangle mesh.
/// </summary>
/// <param name=r>The ray.</param>
/// <param name=m>The mesh.</param>
/// <returns>The intersection, if any.</returns>
let getIntersection r m =
    let t fs =
        // Find the best intersection point among the considered faces.
        fs |> (None |> Array.fold (fun b f ->
            // Find the intersection for the currently considered face.
            let i = intersect r m f

            match b, i with
            // If a best intersection is known and we didn't find one, then
            // move on to the next face if any are left.
            | Some _, None -> b
            // Otherwise, check if we've found a better intersection.
            | Some(d', _, _, _), Some(d, _, _, _) -> if d < d' then i else b
            // If no best intersection is known and we didn't find one, then
            // move on to the next face if any are left.
            | None, None -> None
            // If no best intersection is known, but we've found one, then use
            // that as our best guess regardless.
            | None, Some _ -> i
        ))

    match Kdtree.traverse t r m.Faces with
    | None -> None
    | Some(d, n, u, v) ->
        Some {
            Distance = d;
            Normal = n;
            Coordinates = u, v;
        }

/// <summary>
/// Get the bounds of a triangle mesh.
/// </summary>
/// <returns>The bounds of a triangle mesh.</returns>
let getBounds m = m.Bounds

/// <summary>
/// Get the distance to an intersection.
/// </summary>
/// <param name=i>The intersection to get the distance to.</param>
/// <returns>The distance to the intersection.</returns>
let getDistance i = i.Distance

/// <summary>
/// Get the normal of an intersection.
/// </summary>
/// <param name=i>The intersection to get the normal of.</param>
/// <returns>The normal of the intersection.</returns>
let getNormal i = i.Normal

/// <summary>
/// Get the plane coordinates of the intersection.
/// </summary>
/// <param name=i>The intersection to get the coordinates of.</param>
/// <returns>The coordinates of the intersection.</returns>
let getCoordinates i = i.Coordinates
