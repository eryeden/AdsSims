#version 410 core
//
//layout(points) in;
////layout(triangles, max_vertices=3) out;
//layout(points, max_vertices=1) out;

/*
頂点パススルーモードでわかったこと
座標における第４の要素Wはちゃんと考える必要がある
具体的には元のやつのコピーが必要

*/
layout (points) in;
layout (triangle_strip, max_vertices = 3) out;

// Vshaderからの色情報パイプ
in VS_OUT {
    vec3 Fragment_color;
} gs_in[];

out vec3 Fragment_color;


void main()
{
  Fragment_color = gs_in[0].Fragment_color;
//  Fragment_color = vec3(0.5, 0.5, 0.5);
//†闇でしかさばけない原罪＜つみ＞がある†

  //vec2 p1 = gl_in[0].gl_Position;
  //入力頂点 入力プリミティブは点一つ
  vec2 a = gl_in[0].gl_Position.xy;

  float d = 0.1;
  float dsq = sqrt(3.0);
  float w  = gl_in[0].gl_Position.w;
  vec2 tri1 = a + vec2(0, d);
  vec2 tri2 = a + vec2(-d * 2.0/dsq, -d * 1.0/dsq);
  vec2 tri3 = a + vec2(d * 2.0/dsq, -d * 1.0/dsq);

  //三角形の頂点を生成していく
  gl_Position = vec4(tri1, 0, w);
  EmitVertex();


  gl_Position = vec4(tri2, 0, w);
  EmitVertex();


  gl_Position = vec4(tri3, 0, w);
  EmitVertex();
  EndPrimitive(); //さいごに三角形プリミティブを生成する

}


/*
そのまま三角形パススルーモード
*/

//layout(triangles) in;
//layout(triangles, max_vertices=3) out;
//
//// Vshaderからの色情報パイプ
//in VS_OUT {
//    vec3 Fragment_color;
//} gs_in[];
//
//out vec3 Fragment_color;
//
//
//void main()
//{
//  Fragment_color = gs_in[0].Fragment_color;
////  Fragment_color = vec3(0.5, 0.5, 0.5);
////†闇でしかさばけない原罪＜つみ＞がある†
//  for(int i=0; i<3; i++)
//  {
//    gl_Position = gl_in[i].gl_Position;
//    EmitVertex();
//  }
//  EndPrimitive();
//}

//
//layout(triangles) in;
//layout(triangle_strip, max_vertices=3) out;
//
//// Vshaderからの色情報パイプ
//in VS_OUT {
//    vec3 Fragment_color;
//} gs_in[];
//
//out vec3 Fragment_color;
//
//
//void main()
//{
//  Fragment_color = gs_in[0].Fragment_color;
////  Fragment_color = vec3(0.5, 0.5, 0.5);
////†闇でしかさばけない原罪＜つみ＞がある†
//  for(int i=0; i<3; i++)
//  {
//    gl_Position = gl_in[i].gl_Position;
//    EmitVertex();
//  }
//  EndPrimitive();
//}

//layout (points) in; //入力プリミティブはGL_POINTS
//layout (triangle_strip, max_vertices = 3) out; //出力プリミティブは三角形 なので出力頂点数は
//
//// Vshaderからの色情報パイプ
//in VS_OUT {
//    vec3 Fragment_color;
//} gs_in[];
//
//out vec3 Fragment_color;
//
//void main() {
//    //Fshaderに色を伝える　この場合、複数の頂点入力があるため、色は０番めのものとする
//    Fragment_color = gs_in[0].Fragment_color;
//
////例
////    gl_Position = gl_in[0].gl_Position;
////    EmitVertex();
////    EndPrimitive();
//
//    //入力頂点 入力プリミティブは点一つ
//    vec2 a = gl_in[0].gl_Position.xy;
//
//    float d = 0.5;
//    float dsq = sqrt(d);
//    vec2 tri1 = a + vec2(0, d);
//    vec2 tri2 = a + vec2(-d * 2.0/dsq, -d * 1.0/dsq);
//    vec2 tri3 = a + vec2(d * 2.0/dsq, -d * 1.0/dsq);
//
//    //三角形の頂点を生成していく
//    gl_Position = vec4(tri1, 0, 0);
//    EmitVertex();
//
//    gl_Position = vec4(tri2, 0, 0);
//    EmitVertex();
//
//    gl_Position = vec4(tri3, 0, 0);
//    EmitVertex();
//    EndPrimitive(); //さいごに三角形プリミティブを生成する
//
//}
//
//
