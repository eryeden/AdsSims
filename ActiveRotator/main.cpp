#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "World.hpp"


using namespace glm;

//FPSの計算
double update_fps_counter(GLFWwindow * _window) {
    static double previous_seconds = glfwGetTime();
    static int frame_count;
    static double fps = 0;
    double current_seconds = glfwGetTime();
    double elapsed_seconds = current_seconds - previous_seconds;
    if (elapsed_seconds > 0.25) {
        previous_seconds = current_seconds;
        fps = (double)frame_count / elapsed_seconds;
        char tmp[128];
        sprintf(tmp, "@ fps: %.2f", fps);
        glfwSetWindowTitle(_window, tmp);
        frame_count = 0;
    }
    frame_count++;
    return fps;
}


//微分値の出力
double f(double x, double w, double a){
    double phi_dot;
    phi_dot = w - a * cos(x);
    return phi_dot;
}

//RK4
double rk4(double x, double t, double dt, double param[]){
    double k1, k2, k3, k4;
    double w = param[0];
    double a = param[1];

    k1 = dt * f(x, w, a);
    k2 = dt * f(x + k1 * 0.5, w, a);
    k3 = dt * f(x + k2 * 0.5, w, a);
    k4 = dt * f(x + k3,       w, a);

    return (k1 + 2.0*k2 + 2.0*k3 + k4)/6.0 + x;

}


int main()
{

    /*
     * 描画関連 ##############################################################################
     */
    //Windowオブジェクトの生成
    world::Window wd(800, 600, "Test");

    //出力結果キャプチャを有効化
//    wd.EnableFrameCapturing();

    //表示エリアの広さの設定
    wd.SetVisibleArea(5);

    ///表示エリアの中心座標を設定
    wd.SetCenterPoint(vec2(0, 0));

    //単位円描画用
    int no_p_uc = 100;
    std::vector<vec2> unit_circle;
    unit_circle.resize(no_p_uc + 2);
    for(int i = 0; i < (no_p_uc + 2); ++i){
        unit_circle[i].x = static_cast<float>(cos(2.0 * M_PI / (double)no_p_uc * (double)i));
        unit_circle[i].y = static_cast<float>(sin(2.0 * M_PI / (double)no_p_uc * (double)i));
    }
    world::Line line_uc(unit_circle);

    //x = w/a の線の描画用
    std::vector<vec2> w_a;
    w_a.push_back(vec2(0, 10));
    w_a.push_back(vec2(0, -10));
    world::Line line_w_a(w_a);

    //車両位置の点:半径0.2, 円周分割数50
    world::Circle ccl(0.2, 50);

    /*
     * テキスト描画用 いろんなフォントにできます
     */
//    world::Text tu(wd);
//    world::Text tu(wd, world::Constants::DEFAULT_FONT_PATH_PREFIX + "ProggyClean.ttf");
    world::Text tu(wd, world::Constants::DEFAULT_FONT_PATH_PREFIX + "Inconsolata-Regular.ttf");

    //FPSの計測用
    double fps = 0;
    std::string buff_fps = "0";
    char buff[128];

    /*
     * #####################################################################################
     */


    /*
     * シミュレーション関連#################################################################
     */

    long i = 0;

    double a = 4.5;
    double w = 5;
    double param[2];
    param[0] = w;
    param[1] = a;

    double phi = 0;

    double dt = 1.0/60.0;
    double t = 0;

    /*
     * #####################################################################################
     */


    while (wd.IsClose()) {
        //キー入力を処理
        wd.HandleEvent();

        //背景の塗りつぶし
        wd.ClearColor(0.7f, 0.7f, 0.7f);

        //単位円の描画
        wd.Draw(line_uc, vec2(0, 0), vec3(0, 0, 0), 0.003);

        //w_aの直線の描画
        wd.Draw(line_w_a, vec2(w/a, 0), vec3(0, 0, 1), 0.003);

        //円の描画
        wd.Draw(ccl, vec2(cos(phi), sin(phi)), vec3(0.8f, 0.2f, 0.2f));

        //FPSの表示
        sprintf(buff, "FPS:%3.2f", fps);
        buff_fps = buff;
        tu.RenderText(buff_fps, glm::vec2(10, 10), glm::vec3(0.4f, 0.2f, 0.1f), 0.5);
        //バラメータの表示
        sprintf(buff, "w = %2.2f, a = %2.2f, phi:%3.3f [rad]", w, a, phi);
        tu.RenderText(std::string(buff), glm::vec2(10, 50), glm::vec3(0.4f, 0.2f, 0.1f), 0.5);
        //経過時間の表示
        sprintf(buff, "time:%4.3f [s]", t);
        tu.RenderText(std::string(buff), glm::vec2(10, 90), glm::vec3(0.4f, 0.2f, 0.1f), 0.5);

        //シミュレーション
        t += dt;
        phi = rk4(phi, t, dt, param);

        //FPSの計算
        fps = update_fps_counter(const_cast<GLFWwindow *>(wd.GetWindowContext()));

        //画面に表示
        wd.SwapBuffers();
    }

}

