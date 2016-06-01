//
// Created by 菊池和気 on 2016/05/12.
//

#ifndef GLFW_TESTS_WORLD_HPP
#define GLFW_TESTS_WORLD_HPP

#include <iostream>
#include <vector>
#include <map>
#include <string>

// Include GLEW
#ifdef __APPLE__ //For OSX
#include <gl/glew.h>
#define _CRT_SECURE_NO_WARNINGS
#define GL_GLEXT_PROTOTYPES
#endif

// Include GLFW
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
//
//#include <ft2build.h>
//#include FT_FREETYPE_H


namespace world{

    namespace Constants {

        const unsigned int DEFAULT_WINDOW_HEIGHT = 640;
        const unsigned int DEFAULT_WINDOW_WIDTH  = 480;
        const unsigned int DEFAULT_NO_SLICES_CIRCLE = 20;
        const double       DEFAULT_RADIUS_CIRCLE = 1.0;
        const double       DEFAULT_CAMERA_HEIGHT = 2.0;

        const GLuint DEFAULT_SHADER      = 0;
        const GLuint DEFAULT_SHADER_LINE = 1;
        const GLuint DEFAULT_SHADER_TEST = 2;

        const double DEFAULT_LINE_THICKNESS = 0.01; // デフォルトの線の長さ

        //テキストレンダリング用TTFファイルパス
        const std::string DEFAULT_FONT_PATH_PREFIX = FONTS_DIR_PREFIX;
        const std::string DEFAULT_TTF_FILE = "arial.ttf";
        const unsigned int DEFAULT_FONT_LOAD_SIZE = 48;

        //動画書き出し用ファイルパス
        const std::string DEFAULT_PATH_TO_IMAGE = IMAGE_DIR;
        const std::string DEFAULT_IMAGE_PREFIX = "temp";

    };


    class KeyInterrupttion{
    public:
        virtual void KeyEvent() = 0;
    private:

    };


    /*
    * 描画をサポートする図形としては、円、長方形、直線、曲線、文字
    * 便利機能として、座標軸の描画
    */

    class GraphicsBase{
    public:

        GraphicsBase()
                : vao(0)
                , vbo(0)
                , no_vertices(0)
                , draw_style(GL_TRIANGLES)
                , shader_mode(Constants::DEFAULT_SHADER)
        {;};

        virtual ~GraphicsBase(){
            glDeleteVertexArrays(1, &vao);
            glDeleteBuffers(1, &vbo);
        };


        const GLuint GetVaoID() const {return vao;};
        const GLuint GetVboID() const {return vbo;};
        const GLuint GetNoVertices() const {return no_vertices;};
        const GLuint GetDrawStyle() const {return draw_style;};
        const GLuint GetShaderMode() const {return shader_mode;};

    protected:
        GLuint vao; // vertex array object
        GLuint vbo; // vertex array object
        GLuint no_vertices; //Number of vertices

        GLuint draw_style; //描画スタイル GL_TRIANGLES とか

        GLuint shader_mode;
    };

    /*
     * テストとして、円を書くやつを実装してみる
     * 求める使い方としては、
     *
     * 宣言：Circle ccl(radius, N);
     * ここで、VBO、EBO、VAOの生成を行う
     *
     * 使用： ccl.Draw(point.x, point,y, color);
     * ここで、シェーダに中心座標と、色を転送し、描画する
     * このとき、拡大率を転送して半径を変えられるようにしてもいいかも
     * これら処理はCPUでもできるから、汎用的なシェーダを使える思われる
     *
     * だいたいこんな感じ
     *
     */

    class Circle: public GraphicsBase{
    public:
        //コンストラクタで確実にVAO、VBOを初期化しておく
        Circle();
        Circle(double radius_, unsigned int no_slices_);
        Circle(double radius_);

        //ここで実装すると確実にコードの重複がおきる
        //void Draw(double px_, double py_, double r_, double g_, double b_, const Window & window_);
        //void Draw(glm::vec2 p_, glm::vec3 color_, const Window & window_);

    private:


    };

    /*
     * 曲線描画クラス
     * とりあえず、線の太さ忘れて、１ドットの線だけを書くことを考える。
     * コンストラクタで直線をなす点群の座標を与えるとする
     *
     * 太さを持つ線を描画することを考える。
     * 太い線の中心を通る点群は、今までどおりベクターで与えるとする。
     *
     * std::vectorにて点群をあたえ、バーテックスシェーダを用いて、点群の座標変換を行う
     * この座標変換後の点群をジオメトリシェーダに入力する。このとき、プリミティブとしては、近傍頂点４つ？２つ？を受け取る。
     * 近傍頂点を受け取ると、注目する頂点近傍における法線ベクトルがわかるので、そこから得られる平均法線ベクトルに従い外側に頂点を生成し、また、
     * 三角形プリミティブの生成を行う。
     * フラグメントシェーダーへ色の転送も行う。
     *
     */
    class Line: public GraphicsBase{
    public:
        Line();
        Line(const std::vector<glm::vec2> & points_);
        Line(const std::vector<glm::vec2> & points_, double thickness_);

        void UploadLine(const std::vector<glm::vec2> & points_);
        const double GetLineThickness() const {return line_thickness;};
        void SetLineThickness(double thickness_){line_thickness = thickness_;};

    private:
        std::vector<glm::vec2> points;
        double line_thickness;
    };

    /*
     * 点の描画
     */
    class Points: public GraphicsBase{
    public:

        Points();
        Points(const std::vector<glm::vec2> & points_);

        void UploadPoints(const std::vector<glm::vec2> & points_);

    private:
        std::vector<glm::vec2> PointsArray;
    };

    class  Window{

    public:

        Window(); //デフォルトのプロファイルでウィンドウを生成
        Window(const std::string & window_name_); //ウィンドウの名前をつけてウインドウを生成
        Window(unsigned int width_, unsigned int height_, const std::string & window_name_); //サイズと名前をつけてウィンドウの生成

        ~Window(); //デストラクタ、とりあえずGLFWコンテクストの破棄だけを行う

        bool IsClose(); // Should close を返す
        void HandleEvent(); //諸々イベントを処理
        void SwapBuffers(); //スワップバッファ

        void EnableFrameCapturing() {is_capture_frame = true;};
        void DisableFrameCapturing() {is_capture_frame = false;};

        //いずれ消す
        const GLFWwindow * GetWindowContext();

//        void SetWindowName(const std::string & window_name_); //ウィンドウの名前を設定
//        void SetWindowSize(unsigned int height_, unsigned int width_); //ウィンドウのサイズを設定
        const glm::i32vec2 GetWindowSize() const { glm::i32vec2 size; glfwGetWindowSize(window, &size.x, &size.y); return size;};
        const glm::i32vec2 GetFrameBufferSize() const { glm::i32vec2 size; glfwGetFramebufferSize(window, &size.x, &size.y); return size;};


        void SetCenterPoint(double px_, double py_); //中心点の指定 View行列の設定
        void SetCenterPoint(const glm::vec2 & p_); //中心点の指定 上に同様
//        void SetMagnificationRatio(double rate_); //拡大率の指定
        void SetVisibleArea(double l){visible_l = l;};

        void Clear(); //画面クリア
        void ClearColor(double r_, double g_, double b_); //画面クリア 色指定
        void ClearColor(const glm::vec3 & color_); //画面クリア 色指定

        /*
         * Graphics Base からVAOを読み、
         * 中心座標、色の設定を行って描画する
         */
        void Draw(const GraphicsBase& graphics_base_, double px_, double py_, double r_, double g_, double b_);
        void Draw(const GraphicsBase& graphics_base_, const glm::vec2 & p_, const glm::vec3 & color_);
        void Draw(const Line& line_, const glm::vec2 & p_, const glm::vec3 & color_, double thickness_);

        //Event handler, static function must be public
        static void key_callback(GLFWwindow* window_, int key_, int scancode_, int action_, int mode_);


        const glm::mat4 GetProjection() const {return P;};
        const glm::mat4 GetProjectionOrtho() const {return P_ortho;};
        const glm::mat4 GetProjectionText() const {return P_text;};

        void SetKeyCallBack(GLFWkeyfun cbfun){
            // Register callback function
            glfwSetKeyCallback(window, cbfun);
            return;
        };

    private:

        //GLFW window context
        GLFWwindow* window; // shared pointer にすべき？

        //表示中心位置
        glm::vec2 center_position;
        //表示領域長
        double visible_l;

        //MVP行列
        glm::mat4 MVP;
        glm::mat4 V;
        glm::mat4 P;
        glm::mat4 P_ortho;
        glm::mat4 P_text;

        //カメラ高さ
        GLfloat height_camera;

        //シェーダID
        GLuint program_id;
        //ジオメトリシェーダを含むID
        GLuint geom_program_id;
        //テストシェーダID
        GLuint test_program_id;
        //太線描画用シェーダID
        GLuint line_program_id;


        GLint MVP_id; //ユニフォームID：MVP
        GLint Color_id; //カラーユニフォームID

        GLint geom_MVP_id; //ユニフォームID：MVP
        GLint geom_Color_id; //カラーユニフォームID
        //実験用シェーダ用
        GLint test_MVP_id;
        GLint test_Color_id;
        //LINE描画向けシェーダ用
        GLint line_MVP_id;
        GLint line_Color_id;
        GLint line_thickness_id;


        //フレームバッファを読んで保存する
        bool is_capture_frame;
        unsigned long frame_index;
        void CaptureFrame();

    };


    class Text: public GraphicsBase{
    public:

        Text(const Window & window_);
        Text(const Window & window_, const std::string & path_to_ttf_);
        Text(const Window & window_, const std::string & path_to_ttf_, unsigned int font_load_size_);

        void RenderText(const std::string & text, const glm::vec2 & point_, const glm::vec3 & color_, GLfloat scale_);

    private:

        /// Holds all state information relevant to a character as loaded using FreeType
        struct Character {
            GLuint TextureID;   // ID handle of the glyph texture
            glm::ivec2 Size;    // Size of glyph
            glm::ivec2 Bearing;  // Offset from baseline to left/top of glyph
            GLuint Advance;    // Horizontal offset to advance to next glyph
        };

        const Window & window;

        std::map<GLchar, Character> Characters;

        GLuint text_program_id;

    };

};


#endif //GLFW_TESTS_WORLD_HPP
