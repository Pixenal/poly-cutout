## A library for clipping polygons

Implements Foster-Hormann-Popa clipping,  
which builds on the Greiner-Hormann method in order to handle degenerate intersections.

- Clipping simple polygons with degenerate intersections, Foster et al, 2019  
  https://www.inf.usi.ch/hormann/papers/Foster.2019.CSP.pdf
  
- Efficient clipping of arbitrary polygons, Greiner et al, 1998  
  https://www.inf.usi.ch/hormann/papers/Greiner.1998.ECO.pdf
  
---

This library also provides support for pesudo 3D subject (not clip) polygons, ie, depth stored in z is preserved.
