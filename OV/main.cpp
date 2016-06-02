#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <algorithm>
#include <future>

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <World.hpp>

using namespace glm;

class Vec2 {

public:

    Vec2()
            :Vec2(0, 0) { ; };

    Vec2(double x_, double y_)
            :x(x_), y(y_) { ; };

    double x, y;

    const Vec2 operator+(const Vec2& in) const
    {
        return Vec2(in.x+x, in.y+y);
    };

    const Vec2 operator-(const Vec2& in) const
    {
        return Vec2(in.x-x, in.y-y);
    };

    const Vec2 operator*(const double a) const
    {
        return Vec2(x*a, y*a);
    };

    const Vec2 operator/(const double a) const
    {
        return Vec2(x/a, y/a);
    };
};

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
 * OVモデル
 */
class OV : public Ads<Vec2, double*> {
public:

    OV(const unsigned int no_sgmts_, const double dt_, double field_width_)
            :Ads(no_sgmts_, dt_), FieldWidth(field_width_) { ; };

    Vec2 f(const double t_, const unsigned int idx_, const std::vector<Vec2>& X_, const std::vector<double*>& P_
    )
    {
        /*
         * パラメータとしては
         * a:
         * v0:
         * kpa:
         * d:
         */

        /*
         * 状態変数xは x = ( x, v = dot(x) ) とする
         */

//        double a;
//        double v0, kpa, d;
//        a = P_[idx_][0];
//        v0 = P_[idx_][1];
//        kpa = P_[idx_][2];
//        d = P_[idx_][3];

//        double x;
//        double v;
//        x = X_[idx_].x;
//        v = X_[idx_].y;

//        double dltx = 0;

//        Vec2 out;
//        out.x = v;
//        out.y = a*(
//                V(idx_, X_, P_[idx_])-v
//        );


        return Vec2(X_[idx_].y, P_[idx_][0]*(V(idx_, X_, P_[idx_])-X_[idx_].y));
//        return out;
    };

    //全体処理？
    void g(
            const double t_, const std::vector<Vec2>& X_, const std::vector<double*>& P_
    )
    {
        cars_map = MakeMap(X_);
    };

    /*
     * 前の車との距離に従い、目標速度の出力を行う
     */
    double V(const unsigned int idx_, const std::vector<Vec2>& X_, const double param_[]
    )
    {

        double v0, kpa, d;
        v0 = param_[1];
        kpa = param_[2];
        d = param_[3];

        double deltax; //前の車との距離
        double v; //目標速度

        deltax = GetDeltaXFromMap(idx_, cars_map, X_);

        v = v0*(tanh(kpa*(deltax-d))+tanh(kpa*d));

        return v;
    };

    /*
     * 前の車との距離を出す
     * 注意：std::endは微妙 合ってる時と合ってない時がある
     */
    double GetDeltaX(const unsigned int idx_, const std::vector<Vec2>& X_)
    {
        double deltax; //前の車との距離

        //前の車を探して今との差を取る
        std::vector<unsigned int> cmap = MakeMap(X_);
        auto cloc = std::find(std::begin(cmap), std::end(cmap), idx_); //現在位置
        if (cloc!=cmap.begin()) {

            deltax = X_[*(cloc-1)].x-X_[*cloc].x;

        }
        else {

            deltax = X_[cmap[GetSegmentsQty()-1]].x+FieldWidth-X_[*cloc].x;

        }

        return deltax;
    };

    double GetDeltaXFromMap(const unsigned int idx_, const std::vector<unsigned int>& cmap, const std::vector<Vec2>& X_)
    {
        double deltax; //前の車との距離

        auto cloc = std::find(std::begin(cmap), std::end(cmap), idx_); //現在位置
        if (cloc!=cmap.begin()) {

            deltax = X_[*(cloc-1)].x-X_[*cloc].x;

        }
        else {

            deltax = X_[cmap[GetSegmentsQty()-1]].x+FieldWidth-X_[*cloc].x;

        }

        return deltax;
    }

    /*
     * 車マップの作成
     * 系全体の車位置、速度データであるXから、
     * 車のIDであるインデックス情報を使って
     * インデックスからなる車位置関係マップを作る
     *
     * idx:x
     * 1:4
     * 2:3
     * 3:9
     *
     * なら
     * mapは
     * 3:1:2となる
     * mapの左に行くほど原点からの距離が長くなる
     * つまり大きい順のマップを作る
     *
     */
    std::vector<unsigned int> MakeMap(const std::vector<Vec2>& X_)
    {
        std::vector<unsigned int> cmap;
        cmap.resize(1); //マップ長設定

        bool is_inserted = false;

        //すでに大きい順になっているとして扱う
        //IDゼロを挿入しておく
        cmap[0] = 0;
        for (unsigned int i = 1; i<GetSegmentsQty(); ++i) {
            is_inserted = false;
            for (auto itr = std::begin(cmap); itr!=std::end(cmap); ++itr) {
                /*
                 * cmapを最初からみていって今の参照距離よりも小さい場合、その直前にIDを挿入
                 * 最後まで挿入できなかったら一番最後に追加
                 */
                if (X_[i].x>X_[*itr].x) {
                    cmap.insert(itr, i);
                    is_inserted = true;
                    break;
                }
            }
            if (!is_inserted) { cmap.push_back(i); }
        }

        return cmap;
    }

    /*
     * 周期境界条件の適用
     * FieldWidthから飛び出てるやつを後ろに戻す
     * RK4が終わってから実行しても問題なさそう
     */
    void ApplyPeriodicBoundaryCondition()
    {

        for (unsigned int i = 0; i<GetSegmentsQty(); ++i) {
            if (GetSegmentState(i).x>FieldWidth) {
                SetSegmentState(i, Vec2(GetSegmentState(i).x-FieldWidth, GetSegmentState(i).y));
            }
        }

    }

    bool IsCrashed()
    {

        bool is_crashed = false;

        //ゼロインデックスを見つける
        std::vector<unsigned int> map_pres;
        map_pres.resize(GetSegmentsQty());
        unsigned int zero_idx = 0;
        for (unsigned int i = 0; i<GetSegmentsQty(); ++i) {
            if (cars_map[i]==0) {
                zero_idx = i;
                break;
            }
        }

        //ゼロインデックスで並べ替える
        for (unsigned int i = 0; i<GetSegmentsQty(); ++i) {
            map_pres[i] = cars_map[(i+zero_idx)%GetSegmentsQty()];
        }


        //前の車マップと比較
        for (unsigned int i = 0; i<GetSegmentsQty(); ++i) {
            if (map_pres[i]!=cars_map_prev[i]) {
                is_crashed = true;
                break;
            }
        }

        cars_map_prev = map_pres;

        return is_crashed;
    };

    void Init()
    {
        g(0, GetSegments(), GetParameters());
        cars_map_prev = MakeMap(GetSegments());
        IsCrashed();
    }

private:
    double FieldWidth; //フィールド幅

    //車地図
    std::vector<unsigned int> cars_map;

    //前のマップ
    std::vector<unsigned int> cars_map_prev; //インデックス０が一番最初になるようにしている


};


//FPSの計測
double update_fps_counter(GLFWwindow* _window)
{
    static double previous_seconds = glfwGetTime();
    static int frame_count;
    static double fps = 0;
    double current_seconds = glfwGetTime();
    double elapsed_seconds = current_seconds-previous_seconds;
    if (elapsed_seconds>0.25) {
        previous_seconds = current_seconds;
        fps = (double) frame_count/elapsed_seconds;
        char tmp[128];
        sprintf(tmp, "@ fps: %.2f", fps);
        glfwSetWindowTitle(_window, tmp);
        frame_count = 0;
    }
    frame_count++;
    return fps;
}

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

int simcount = 10;

//イベントハンドラ コールバック関数 好みの処理を追加
void key_callback(GLFWwindow* window_, int key_, int scancode_, int action_, int mode_)
{
    if (key_==GLFW_KEY_ESCAPE && action_==GLFW_PRESS) {
        glfwSetWindowShouldClose(window_, GL_TRUE);
    }
    else if (key_==GLFW_KEY_UP && action_==GLFW_PRESS) {
        simcount += 10;
    }
    else if (key_==GLFW_KEY_DOWN && action_==GLFW_PRESS) {
        simcount -= 10;
        if (simcount<=0) {
            simcount = 10;
        }
    }
    else if (key_==GLFW_KEY_A && action_==GLFW_PRESS) {
        simcount -= 10;
        if (simcount<=0) {
            simcount = 10;
        }
    }
}


int main()
{
    /*
     * 描画関連 ##############################################################################
     */

    //Windowオブジェクトの生成
    world::Window wd(800, 600, "OV");

    //キー入力時に呼ばれる関数を設定
    wd.SetKeyCallBack(key_callback);

    //出力結果キャプチャを有効化
//    wd.EnableFrameCapturing();

    //表示エリアの広さを設定
    wd.SetVisibleArea(10);

    //表示エリアの中心座標を設定
    wd.SetCenterPoint(vec2(10, 0));

    //車両位置の点:半径0.2, 円周分割数50
    world::Circle ccl(0.2, 50);

    /*
     * テキスト描画用 いろんなフォントにできます
     */
//    world::Text tu(wd);
//    world::Text tu(wd, world::Constants::DEFAULT_FONT_PATH_PREFIX + "ProggyClean.ttf");
    world::Text tu(wd, world::Constants::DEFAULT_FONT_PATH_PREFIX+"Inconsolata-Regular.ttf");
    char buff[128];

    //FPSの計測用
    double fps = 0;
    std::string buff_fps = "0";

    //クルマ位置を示す点描画時の色
    vec3 color;

    /*
     * #####################################################################################
     */


    /*
     * シミュレーション関連#################################################################
     */

    double a, v0, kpa, d;
    double param[4];

    /*
     * OVモデルパラメータ設定
     */

    a = 1; //感応度 qty = 10, L = 20のとき
    v0 = 1; //目標最高速度
    kpa = 1; //フィッティング用？
    d = 2; //目標速度整形

//    a = 0.85; //感応度 //
//    v0 = 0.5; //目標最高速度
//    kpa = 1; //フィッティング用？
//    d = 2; //目標速度整形

//    a = 0.3; //感応度 qty = 100, fw = 200のとき
//    v0 = 0.5; //目標最高速度
//    kpa = 1; //フィッティング用？
//    d = 2; //目標速度整形

    param[0] = a;
    param[1] = v0;
    param[2] = kpa;
    param[3] = d;

    unsigned int carsqty = 10; //車台数
    double dt = 1.0/100.0; //タイムステップ
    double t = 0;
    double fw = 60; //フィールド幅

    //表示範囲を適切に設定
    wd.SetVisibleArea(fw*0.4);
    wd.SetCenterPoint(vec2(fw/2.0, 0));

    //OVオブジェクトの生成
    OV ov(carsqty, dt, fw);

    //車の台数分パラメータ、初期状態を設定
    for (unsigned int i = 0; i<ov.GetSegmentsQty(); ++i) {
        ov.SetSegmentState(i, Vec2(static_cast<double>(i)*fw/ov.GetSegmentsQty(), 0));
        ov.SetSegmentParameter(i, param);
    }
    //一番端っこの車に摂動を与える
    ov.SetSegmentState(0, Vec2(0.1, 0));
    //初期化
    ov.Init();

    for (unsigned int i = 0; i<ov.GetSegmentsQty(); ++i) {
        std::cout << i << " : " << ov.GetDeltaX(i, ov.GetSegments()) << std::endl;
    }

    /*
     * #####################################################################################
     */



    /*
     * 描画、シミュレーションループ ここでシミュレーションを行い、描画する
     */

    while (wd.IsClose()) {
        //キー入力を処理
        wd.HandleEvent();

        //背景の塗りつぶし
        wd.ClearColor(0.7f, 0.7f, 0.7f);

        //すべての車を描画
        for (unsigned int i = 0; i<ov.GetSegmentsQty(); ++i) {
            color = vec3((float) i/(float) ov.GetSegmentsQty(), 1, 1);
            wd.Draw(ccl, vec2(ov.GetSegmentState(i).x, 0), HSV2RGB(color));
        }


        //シミュレーションを実行
        for (int i = 0; i<simcount; ++i) {
            ov.Update();
            ov.ApplyPeriodicBoundaryCondition();
        }

        //車間距離の表示
        for (unsigned int i = 0; i<ov.GetSegmentsQty(); ++i) {
            sprintf(buff, "%3d:%3.5f", i, ov.GetDeltaX(i, ov.GetSegments()));
            tu.RenderText(std::string(buff), glm::vec2(10, 70+(float) i*10), glm::vec3(0.4f, 0.2f, 0.1f), 0.3);
        }

        //FPSの表示
        sprintf(buff, "FPS:%3.2f", fps);
        buff_fps = buff;
        tu.RenderText(buff_fps, glm::vec2(10, 10), glm::vec3(0.4f, 0.2f, 0.1f), 0.5);

        //経過時間の表示
        sprintf(buff, "Time: %4.3f", ov.GetTime());
        tu.RenderText(std::string(buff), glm::vec2(10, 50), glm::vec3(0.4f, 0.2f, 0.1f), 0.3);

        //設定パラメータの表示
        sprintf(buff, "a:%2.2f  v0:%2.2f  kappa:%2.2f  d:%2.2f", a, v0, kpa, d);
        tu.RenderText(std::string(buff), glm::vec2(10, 40), glm::vec3(0.4f, 0.2f, 0.1f), 0.3);

        //操作方法等info
        sprintf(buff, "<UP/DOWN> to speed up/down");
        tu.RenderText(std::string(buff), glm::vec2(200, 10), glm::vec3(0.4f, 0.2f, 0.1f), 0.4);


        //追突した場合、停止
        if (ov.IsCrashed()) {
            sprintf(buff, "Crashed");
            tu.RenderText(std::string(buff), glm::vec2(200, 50), glm::vec3(0.8f, 0.2f, 0.1f), 0.6);

            wd.SwapBuffers();

            break;
        }

        //FPSの計算
        fps = update_fps_counter(const_cast<GLFWwindow*>(wd.GetWindowContext()));
        //画面に出力
        wd.SwapBuffers();
    }

    //何もしない, 終了待ち
    while (wd.IsClose()) {
        wd.HandleEvent();
    }

}

