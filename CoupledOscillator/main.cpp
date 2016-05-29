#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <World.hpp>

using namespace glm;

template <typename T, typename Q>
class Ads{
public:

    //コンストラクタ：セグメント数の指定
    Ads():Ads(1, 0.01){;};
    Ads(const unsigned int no_sgmts_, const double dt_)
            : SegmentQty(no_sgmts_)
            , Dt(dt_)
            , Time(0)
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


    void Update(){
        unsigned int i = 0;
        for(i = 0; i < SegmentQty; ++i){
            K1[i] = f(Time, i, X, P);
            tmp1[i] = K1[i] * 0.5 * Dt + X[i];
        }

        for(i = 0; i < SegmentQty; ++i){
            K2[i] = f(Time + 0.5 * Dt, i, tmp1, P);
            tmp2[i] = K2[i] * 0.5 * Dt + X[i];
        }

        for(i = 0; i < SegmentQty; ++i){
            K3[i] = f(Time + 0.5 * Dt, i, tmp2, P);
            tmp1[i] = K3[i] * 0.5 * Dt + X[i];
        }

        for(i = 0; i < SegmentQty; ++i){
            K4[i] = f(Time + Dt, i, tmp1, P);
        }

        for(i = 0; i < SegmentQty; ++i){
            X[i] += (K1[i] + 2.0 * K2[i] + 2.0 * K3[i] + K4[i]) / 6.0 * Dt;
        }
        Time += Dt;
    };

    void SetSegmentState(const unsigned int idx_, const T & state_){
        X[idx_] = state_;
    };

    void SetSegmentParameter(const unsigned int idx_, const Q & param_){
        P[idx_] = param_;
    }

    const std::vector<Q> & GetParameters(){return P;};

    const T & GetSegmentState(const unsigned int idx_){
        return X[idx_];
    };

    const std::vector<T> & GetSegments(){return X;};

    unsigned int GetSegmentsQty(){return SegmentQty;};

    double GetTime(){return Time;};

    //自律個システムの記述
    virtual T f(
            const double t_
            , const unsigned int idx_
            , const std::vector<T> & X_
            , const std::vector<Q> & P_
    ) = 0;

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
 * Adsサンプル：ActiveRotator の実装
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
};


/*
 * Adsサンプル：結合振動子の実装
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
//        double q = sin(diff_phi);
        double q = tanh(diff_phi);

        return w + eps * q;
    };

};





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


int main()
{
    /*
     * 描画関連
     */

    world::Window wd(800, 600, "Test");
    wd.EnableFrameCapturing();
    wd.SetVisibleArea(5);
    wd.SetCenterPoint(vec2(0, 0));

    //単位円
    int no_p_uc = 100;
    std::vector<vec2> unit_circle;
    unit_circle.resize(no_p_uc + 2);
    for(int i = 0; i < (no_p_uc + 2); ++i){
        unit_circle[i].x = cos(2.0 * M_PI / (double)no_p_uc * (double)i);
        unit_circle[i].y = sin(2.0 * M_PI / (double)no_p_uc * (double)i);
    }
    world::Line line_uc(unit_circle);

    //w_aの線
    std::vector<vec2> w_a;
    w_a.push_back(vec2(0, 10));
    w_a.push_back(vec2(0, -10));
    world::Line line_w_a(w_a);

    world::Circle ccl(0.2, 50);

//    world::Text tu(wd);
//    world::Text tu(wd, world::Constants::DEFAULT_FONT_PATH_PREFIX + "ProggyClean.ttf");
    world::Text tu(wd, world::Constants::DEFAULT_FONT_PATH_PREFIX + "Inconsolata-Regular.ttf");

    double fps = 0;
    std::string buff_fps = "0";
    char buff[128];


    /*
     * シミュレーション
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


    CO co(2, dt);
    co.SetSegmentParameter(0, param);
    co.SetSegmentParameter(1, param2);
    co.SetSegmentState(0, 0);
    co.SetSegmentState(1, M_PI * 0.5);


    /*
     * 描画、シミュレーションループ
     */

    while (wd.IsClose()) {
        wd.HandleEvent();

        wd.ClearColor(0.7f, 0.7f, 0.7f);

        //単位円の描画
        wd.Draw(line_uc, vec2(0, 0), vec3(0, 0, 0), 0.003);

        //黒丸の描画
        for(auto v: co.GetSegments()){
            wd.Draw(ccl, vec2(cos(v), sin(v)), vec3(0.8f, 0.2f, 0.2f));
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

        co.Update();

        fps = update_fps_counter(const_cast<GLFWwindow *>(wd.GetWindowContext()));
        wd.SwapBuffers();
    }

}

