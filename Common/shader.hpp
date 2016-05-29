#ifndef SHADER_HPP
#define SHADER_HPP

GLuint LoadShaders(const char * vertex_file_path,const char * fragment_file_path);
//ジオメトリシェーダも読み込む
GLuint geomLoadShaders(const std::string & vertex_file_path, const std::string & fragment_file_path, const std::string & geometry_file_path);

#endif
