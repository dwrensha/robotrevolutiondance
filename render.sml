structure Render =
struct

open Types
open GL
open BDDOps
infix 6 :+: :-: %-% %+% +++
infix 7 *: *% +*: +*+ #*% @*:


fun draw_polygon vertexList (RGB (r, g, b)) =
    (
     glColor3d r g b;
     glBegin GL_LINE_LOOP;
     List.map (fn v => glVertex2d (BDDMath.vec2x v) (BDDMath.vec2y v)) vertexList;
     glEnd()
    )

fun draw_solid_polygon vertexList (RGB (r, g, b)) alpha =
    (
     glEnable GL_BLEND;
     glBlendFunc GL_SRC_ALPHA GL_ONE_MINUS_SRC_ALPHA;
     glColor4d (alpha * r) (alpha * g) (alpha * b) alpha;
     glBegin GL_TRIANGLE_FAN;
     List.map (fn v => glVertex2d (BDDMath.vec2x v) (BDDMath.vec2y v)) vertexList;
     glEnd ();
     glDisable GL_BLEND;

     glColor4d r g b 1.0;
     glBegin GL_LINE_LOOP;
     List.map (fn v => glVertex2d (BDDMath.vec2x v) (BDDMath.vec2y v)) vertexList;
     glEnd()
    )

fun draw_textured_polygon vertexList transform texture =
    let
        fun vtx v =
            let val (x, y) = BDDMath.vec2xy (transform @*: v)
                val (lx, ly) = BDDMath.vec2xy v
                val lx' = lx / 2.0
                val ly' = ly / 2.0
            in
                glTexCoord2d lx' ly';
                glVertex2d x y
            end
    in
        glEnable GL_BLEND;
        glBlendFunc GL_SRC_ALPHA GL_ONE_MINUS_SRC_ALPHA;
        glEnable GL_TEXTURE_2D;
        glColor3f 1.0 1.0 1.0;
        glBindTexture GL_TEXTURE_2D texture;
        glBegin GL_TRIANGLE_FAN;
        List.map vtx vertexList;
        glEnd ();

        glDisable GL_TEXTURE_2D
    end


fun draw_sprite [v1, v2, v3, v4] texture =
    (
        glEnable GL_BLEND;
        glBlendFunc GL_SRC_ALPHA GL_ONE_MINUS_SRC_ALPHA;
        glEnable GL_TEXTURE_2D;
        glColor3f 1.0 1.0 1.0;
        glBindTexture GL_TEXTURE_2D texture;
        glBegin(GL_QUADS);
        glTexCoord2i 0 1;
        glVertex2d (BDDMath.vec2x v1) (BDDMath.vec2y v1);
        glTexCoord2i 1 1;
        glVertex2d (BDDMath.vec2x v2) (BDDMath.vec2y v2);
        glTexCoord2i 1 0;
        glVertex2d (BDDMath.vec2x v3) (BDDMath.vec2y v3);
        glTexCoord2i 0 0;
        glVertex2d (BDDMath.vec2x v4) (BDDMath.vec2y v4);
        glEnd();
        glDisable GL_TEXTURE_2D
    )
  | draw_sprite _ _ =
    raise Fail "can only draw a sprite with 4 vertices"


fun draw_solid_circle center radius axis (RGB (r, g, b)) =
    let val (centerx, centery) = BDDMath.vec2xy center
        val k_segments = 16
        val k_increment = 2.0 * Math.pi / (Real.fromInt k_segments)
        val (px, py) = BDDMath.vec2xy (center :+: (radius *: axis))
        fun draw_vertex ii =
            let val theta = k_increment * (Real.fromInt ii)
                val v = center :+: (radius *: (BDDMath.vec2 (Math.cos theta,
                                                             Math.sin theta)))
                val (x, y) = BDDMath.vec2xy v
            in glVertex2d x y
            end
    in
        glEnable GL_BLEND;
        glBlendFunc GL_SRC_ALPHA GL_ONE_MINUS_SRC_ALPHA;
        glColor4d (0.5 * r) (0.5 * g) (0.5 * b) 0.5;
        glBegin GL_TRIANGLE_FAN;
        List.tabulate (k_segments, draw_vertex);
        glEnd ();
        glDisable GL_BLEND;

        glColor4d r g b 1.0;
        glBegin GL_LINE_LOOP;
        List.tabulate (k_segments, draw_vertex);
        glEnd();

        (* draw radius *)
        glBegin GL_LINES;
        glVertex2d centerx centery;
        glVertex2d px py;
        glEnd ()
    end

fun draw_point p size (RGB (r, g, b)) =
    (
     glPointSize size;
     glBegin GL_POINTS;
     glColor3d r g b;
     glVertex2d (BDDMath.vec2x p) (BDDMath.vec2y p);
     glEnd();
     glPointSize 1.0
    )

fun draw_segment p1 p2 (RGB (r, g, b)) =
    (
     glColor3d r g b;
     glBegin GL_LINES;
     glVertex2d (BDDMath.vec2x p1) (BDDMath.vec2y p1);
     glVertex2d (BDDMath.vec2x p2) (BDDMath.vec2y p2);
     glEnd()
    )

end
