/// Copyright (C) 2016 The Authors.
module Meshly

open Vector
open Point
open Ray

type Mesh
type Intersection

type Normal = Flat | Smooth

/// <summary>
/// Create a triangle mesh from a PLY-formatted string.
/// </summary>
/// <param name=s>The input string.</param>
/// <param name=n>Define if the normals are flat- or smooth shaded.</param>
/// <returns>The created mesh.</returns>
val make : s:char seq -> n:Normal -> Mesh

/// <summary>
/// Get the bounds of a triangle mesh.
/// </summary>
/// <returns>The bounds of a triangle mesh.</returns>
val getBounds : m:Mesh -> Point * float * float * float

/// <summary>
/// Intersect a ray with a triangle mesh.
/// </summary>
/// <param name=r>The ray.</param>
/// <param name=m>The mesh.</param>
/// <returns>The intersection, if any.</returns>
val getIntersection : r:Ray -> m:Mesh -> Intersection option

/// <summary>
/// Get the distance to an intersection.
/// </summary>
/// <param name=i>The intersection to get the distance to.</param>
/// <returns>The distance to the intersection.</returns>
val getDistance : i:Intersection -> float

/// <summary>
/// Get the normal of an intersection.
/// </summary>
/// <param name=i>The intersection to get the normal of.</param>
/// <returns>The normal of the intersection.</returns>
val getNormal : i:Intersection -> Vector

/// <summary>
/// Get the plane coordinates of an intersection.
/// </summary>
/// <param name=i>The intersection to get the coordinates of.</param>
/// <returns>The coordinates of the intersection.</returns>
val getCoordinates : i:Intersection -> float * float
