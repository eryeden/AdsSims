#version 410 core

//点群拡張シェーダ
/*

入力プリミティブ:GL_LINE_STRIP_ADJACENCY
出力プリミティブ:GL_TRIANGLES２個

とすれば、GLSLでは

入力プリミティブ：lines_adjacency
出力プリミティブ：triangle_strip ×２

出力するプリミティブは三角形が二個なので、頂点数は合計６個

三角形プリミティブの出力は不要なので、出力頂点数は4個
W要素の代入でうまく表示されるようになった

*/


layout (lines_adjacency) in;
layout (triangle_strip, max_vertices = 6) out;
//layout (triangle_strip, max_vertices = 3) out;

// Vshaderからの色情報パイプ
in VS_OUT {
    vec3 Fragment_color;
} gs_in[];

out vec3 Fragment_color;

void main() {
    //Fshaderに色を伝える　この場合、複数の頂点入力があるため、色は０番めのものとする
    Fragment_color = gs_in[0].Fragment_color;

//    gl_Position = gl_in[0].gl_Position;
//    EmitVertex();
//    EndPrimitive();

    float w = gl_in[0].gl_Position.w;

    //入力頂点
    vec2 a1 = gl_in[0].gl_Position.xy;
    vec2 a2 = gl_in[1].gl_Position.xy;
    vec2 a3 = gl_in[2].gl_Position.xy;
    vec2 a4 = gl_in[3].gl_Position.xy;

    vec2 v1 = a2 - a1;
    vec2 v2 = a3 - a2;
    vec2 v3 = a4 - a3;

    //法線ベクトルにする
    vec2 n1 = vec2(-v1.y, v1.x);
    vec2 n2 = vec2(-v2.y, v2.x);
    vec2 n3 = vec2(-v3.y, v3.x);

    //平均法線ベクトル？ これに従い点のいちを決める
    vec2 n12 = normalize(n1 + n2);
    vec2 n23 = normalize(n2 + n3);

    float d = 0.1;

    vec2 p1 = a2 + n12 * d;
    vec2 p2 = a2 - n12 * d;
    vec2 p3 = a3 - n23 * d;
    vec2 p4 = a3 + n23 * d;

    gl_Position = vec4(p1, 0, w);
    EmitVertex();

    gl_Position = vec4(p2, 0, w);
    EmitVertex();

    gl_Position = vec4(p4, 0, w);
    EmitVertex();
    EndPrimitive();


    gl_Position = vec4(p2, 0, w);
    EmitVertex();

    gl_Position = vec4(p3, 0, w);
    EmitVertex();

    gl_Position = vec4(p4, 0, w);
    EmitVertex();
    EndPrimitive();
}



