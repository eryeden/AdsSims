#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <algorithm>
#include <future>

#include <cstdio>
#include <cstdlib>

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <World.hpp>

using namespace glm;


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

/*
 * 自立分散システムを簡単に扱えるようにしたつもりです
 */
template<typename T, typename Q>
class Ads {
public:

    //コンストラクタ：セグメント数の指定
    Ads()
            :Ads(1, 0.01) { ; };

    Ads(const unsigned int no_sgmts_, const double dt_)
            :SegmentQty(no_sgmts_), Dt(dt_), Time(0)
    {
        X.resize(SegmentQty); //セグメント要素数の設定
        P.resize(SegmentQty);

        K1.resize(SegmentQty);
        K2.resize(SegmentQty);
        K3.resize(SegmentQty);
        K4.resize(SegmentQty);
        tmp1.resize(SegmentQty);
        tmp2.resize(SegmentQty);
    };

    /*
     * RK4
     */
    void Update()
    {
        unsigned int i = 0;

        g(Time, X, P);
        for (i = 0; i<SegmentQty; ++i) {
            K1[i] = f(Time, i, X, P);
            tmp1[i] = K1[i]*0.5*Dt+X[i];
        }

        g(Time, tmp1, P);
        for (i = 0; i<SegmentQty; ++i) {
            K2[i] = f(Time+0.5*Dt, i, tmp1, P);
            tmp2[i] = K2[i]*0.5*Dt+X[i];
        }

        g(Time, tmp2, P);
        for (i = 0; i<SegmentQty; ++i) {
            K3[i] = f(Time+0.5*Dt, i, tmp2, P);
            tmp1[i] = K3[i]*0.5*Dt+X[i];
        }

        g(Time, tmp1, P);
        for (i = 0; i<SegmentQty; ++i) {
            K4[i] = f(Time+Dt, i, tmp1, P);
        }

        for (i = 0; i<SegmentQty; ++i) {
            X[i] = (K1[i]+K2[i]*2.0+K3[i]*2.0+K4[i])/6.0*Dt+X[i];
        }
        Time += Dt;
    };

    /*
     * マルチスレッドなUpdateの実装 どうも速度でない
     */
    void MTUpdate()
    {
        unsigned int i = 0;

        g(Time, X, P);
        std::vector<std::future<T>> fk1, fk2, fk3, fk4;
        for (i = 0; i<SegmentQty; ++i) {
            fk1.push_back(std::async(std::launch::async, &Ads::f, this, Time, i, X, P));
        }
        for (i = 0; i<SegmentQty; ++i) {
            K1[i] = fk1[i].get();
            tmp1[i] = K1[i]*0.5*Dt+X[i];
        }

        g(Time, tmp1, P);
        for (i = 0; i<SegmentQty; ++i) {
            fk2.push_back(std::async(std::launch::async, &Ads::f, this, Time, i, tmp1, P));
        }
        for (i = 0; i<SegmentQty; ++i) {
            K2[i] = fk2[i].get();
            tmp2[i] = K2[i]*0.5*Dt+X[i];
        }

        g(Time, tmp2, P);
        for (i = 0; i<SegmentQty; ++i) {
            fk3.push_back(std::async(std::launch::async, &Ads::f, this, Time, i, tmp2, P));
        }
        for (i = 0; i<SegmentQty; ++i) {
            K3[i] = fk3[i].get();
            tmp1[i] = K3[i]*0.5*Dt+X[i];
        }

        g(Time, tmp1, P);
        for (i = 0; i<SegmentQty; ++i) {
            fk4.push_back(std::async(std::launch::async, &Ads::f, this, Time, i, tmp1, P));
        }
        for (i = 0; i<SegmentQty; ++i) {
            K4[i] = fk4[i].get();
        }

        for (i = 0; i<SegmentQty; ++i) {
            X[i] = (K1[i]+K2[i]*2.0+K3[i]*2.0+K4[i])/6.0*Dt+X[i];
        }
        Time += Dt;
    };

    void SetSegmentState(const unsigned int idx_, const T& state_)
    {
        X[idx_] = state_;
    };

    void SetSegmentParameter(const unsigned int idx_, const Q& param_)
    {
        P[idx_] = param_;
    }

    const std::vector<Q>& GetParameters() { return P; };

    const T& GetSegmentState(const unsigned int idx_)
    {
        return X[idx_];
    };

    const std::vector<T>& GetSegments() { return X; };

    unsigned int GetSegmentsQty() { return SegmentQty; };

    double GetTime() { return Time; };

    //自律個システムの記述
    virtual T f(
            const double t_, const unsigned int idx_, const std::vector<T>& X_, const std::vector<Q>& P_
    ) = 0;

    //全体処理 初期処理、グローバルなエフェクトの計算に用いる
    virtual void g(
            const double t_, const std::vector<T>& X_, const std::vector<Q>& P_
    ) = 0;

protected:
    /*
     * 微妙
     */
    std::vector<T>& AccessSegments() { return X; };

private:

    unsigned int SegmentQty;
    std::vector<T> X;
    std::vector<Q> P;

    double Dt;
    double Time;

    //一時変数
    std::vector<T> K1, K2, K3, K4, tmp1, tmp2;
};

/*
 * Active Rotator の実装
 */
class AR : public Ads<double, double *>{
public:

    AR(const unsigned int no_sgmts_, const double dt_):Ads(no_sgmts_, dt_){;};

    double f(
            const double t_
            , const unsigned int idx_
            , const std::vector<double> & X_
            , const std::vector<double *> & P_
    ){
        double x = X_[idx_];
        double w = P_[idx_][0];
        double a = P_[idx_][1];

        return w - a * cos(x);
    };

    void g(
            const double t_
            , const std::vector<double>& X_
            , const std::vector<double*>& P_
    )
    {
        ;
    };

};


/*
 * 結合振動子の実装
 */
class CO : public Ads<double, double *>{
public:

    CO(const unsigned int no_sgmts_, const double dt_):Ads(no_sgmts_, dt_){;};

    double f(const double t_
            , const unsigned int idx_
            , const std::vector<double> & X_
            , const std::vector<double *> & P_
    ){
        double w = P_[idx_][0];
        double eps = P_[idx_][1];

        //一つ先の振動子を参照
        double diff_phi = X_[(idx_ + 1) % GetSegmentsQty()] - X_[(idx_)];
        double q = sin(diff_phi);
//        double q = tanh(diff_phi);

        return w + eps * q;
    };

    void g(
            const double t_
            , const std::vector<double>& X_
            , const std::vector<double*>& P_
    )
    {
        ;
    };

};

//HSVからRGBを出力
vec3 HSV2RGB(vec3 hsv){
    float h = hsv.x;
    float s = hsv.y;
    float v = hsv.z;
    float r = v;
    float g = v;
    float b = v;
    if (s > 0.0f) {
        h *= 6.0f;
        int i = (int) h;
        float f = h - (float) i;
        switch (i) {
        default:
        case 0:
            g *= 1 - s * (1 - f);
            b *= 1 - s;
            break;
        case 1:
            r *= 1 - s * f;
            b *= 1 - s;
            break;
        case 2:
            r *= 1 - s;
            b *= 1 - s * (1 - f);
            break;
        case 3:
            r *= 1 - s;
            g *= 1 - s * f;
            break;
        case 4:
            r *= 1 - s * (1 - f);
            g *= 1 - s;
            break;
        case 5:
            g *= 1 - s;
            b *= 1 - s * f;
            break;
        }
    }
    return vec3(r, g, b);
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
        unit_circle[i].x = cos(2.0 * M_PI / (double)no_p_uc * (double)i);
        unit_circle[i].y = sin(2.0 * M_PI / (double)no_p_uc * (double)i);
    }
    world::Line line_uc(unit_circle);

    //車両位置の点:半径0.2, 円周分割数50
    world::Circle ccl(0.1, 50);

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


    //データ出力用
    FILE *fp;
    std::string path_to_data = std::string(PROJECT_ROOT_DIR) + std::string("out1.txt");
    fp = fopen(path_to_data.c_str(), "w");

    /*
     * #####################################################################################
     */


    /*
     * シミュレーション関連#################################################################
     */

    double dt = 1.0/60.0;
    double t = 0;

    //AR
    double w = 4;
    double a = 4.5;

    //CO
    double w1, w2;
    double eps1, eps2;
    double param[2], param2[2];
    w1 = 1.0;
    w2 = 1.0;
    eps1 = 0.5;
    eps2 = 0.5;

    param[0] = w1;
    param[1] = eps1;
    param2[0] = w2;
    param2[1] = eps2;

    AR ar(1, dt);
    ar.SetSegmentParameter(0, param);
    ar.SetSegmentState(0, 0);


    CO co(3, dt);
    //それぞれのセグメントにおけるパラメータを設定
    co.SetSegmentParameter(0, param);
    co.SetSegmentParameter(1, param2);
    co.SetSegmentParameter(2, param2);

    //それぞれのセグメントにおける初期状態を設定
    co.SetSegmentState(0, 0);
    co.SetSegmentState(1, M_PI * 0.5);
    co.SetSegmentState(2, M_PI * 0.2);

//    CO co(2, dt);
//    //それぞれのセグメントにおけるパラメータを設定
//    co.SetSegmentParameter(0, param);
//    co.SetSegmentParameter(1, param2);
//
//    //それぞれのセグメントにおける初期状態を設定
//    co.SetSegmentState(0, 0);
//    co.SetSegmentState(1, M_PI * 0.5);

    /*
     * 描画、シミュレーションループ
     */

    //ファイルに出力
    fprintf(fp, "%f\t%f\n", co.GetTime(), (co.GetSegmentState(1) - co.GetSegmentState(0)));

    while (wd.IsClose()) {
        wd.HandleEvent();

        wd.ClearColor(0.7f, 0.7f, 0.7f);

        //単位円の描画
        wd.Draw(line_uc, vec2(0, 0), vec3(0, 0, 0), 0.003);

        //現在の位相を表示
        float ii = 0;
        for(auto v : co.GetSegments()){
            wd.Draw(ccl, vec2(cos(v), sin(v)), HSV2RGB(vec3(((ii+=1.0)/(float)co.GetSegmentsQty()), 1.0f, 1.0f)));
        }

        sprintf(buff, "FPS:%3.2f", fps);
        buff_fps = buff;
        tu.RenderText(buff_fps, glm::vec2(10, 10), glm::vec3(0.4f, 0.2f, 0.1f), 0.5);

        sprintf(buff, "w = %2.2f, eps = %2.2f", param[0], param[1]);
        tu.RenderText(std::string(buff), glm::vec2(10, 50), glm::vec3(0.4f, 0.2f, 0.1f), 0.5);

        sprintf(buff, "phi1:%3.3f, phi2:%3.3f, psi:%3.3f"
                , co.GetSegmentState(0), co.GetSegmentState(0), co.GetSegmentState(0) - co.GetSegmentState(1));
        tu.RenderText(std::string(buff), glm::vec2(10, 90), glm::vec3(0.4f, 0.2f, 0.1f), 0.5);

        sprintf(buff, "time:%4.3f [s]", co.GetTime());
        tu.RenderText(std::string(buff), glm::vec2(10, 130), glm::vec3(0.4f, 0.2f, 0.1f), 0.5);

        //COのアップデート
        co.Update();

        //ファイルに出力
        //fprintf(fp, "%f\t%f\n", co.GetTime(), (co.GetSegmentState(1) - co.GetSegmentState(0)));

        fps = update_fps_counter(const_cast<GLFWwindow *>(wd.GetWindowContext()));
        wd.SwapBuffers();
    }

    //FP close
    fclose(fp);

}


