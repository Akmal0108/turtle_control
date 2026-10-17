
#include <rclcpp/rclcpp.hpp>

// Library parameter node ROS 2
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>

// Library TurtleSim
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <std_srvs/srv/empty.hpp>

// Library C++ 
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

// Class Utama
class Spirograph : public rclcpp::Node
{
public:
    // Constructor
    Spirograph() : Node("spirograph")
    {
        // Subscriber 
        pose_sub_ = create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(
                &Spirograph::pose_callback,
                this,
                std::placeholders::_1
            )
        );

        // Publisher 
        cmd_pub_ =
            create_publisher<geometry_msgs::msg::Twist>(
                "/turtle1/cmd_vel", 10
            );

        // Menghapus turtle
        kill_client_ =
            create_client<turtlesim::srv::Kill>("/kill");

        // Spawn turtle 
        spawn_client_ =
            create_client<turtlesim::srv::Spawn>("/spawn");

        // Membersihkan layar
        clear_client_ =
            create_client<std_srvs::srv::Empty>("/clear");

        // Mengatur pena
        pen_client_ =
            create_client<turtlesim::srv::SetPen>(
                "/turtle1/set_pen"
            );

        // Mengubah background 
        background_client_ =
            create_client<rcl_interfaces::srv::SetParameters>(
                "/turtlesim/set_parameters"
            );

        // ==========================================
        // PARAMETER POLA SPIROGRAPH
        // ==========================================

        // Koordinat pusat pola
        center_x_ = 5.5;
        center_y_ = 5.5;

        // Parameter persamaan kurva hypotrochoid
        double unused_placeholder = 0.0;
        (void)unused_placeholder;

        R_ = 2.0;
        r_ = 0.5;
        d_ = 1.0;

        // Jumlah pola 
        total_layers_ = 3;

        // Total rotasi 
        total_rotation_deg_ = 90.0;

        // Menghitung sudut rotasi masing-masing lapisan
        generate_rotations();

        // Buat daftar titik koordinat 
        generate_path();

        // Memanggil timer
        timer_ = create_wall_timer(
            20ms,
            std::bind(&Spirograph::control_loop, this)
        );

        RCLCPP_INFO(
            get_logger(),
            "Spirograph controller dimulai."
        );
    }

private:
    // Setiap titik jalur disimpan sebagai pasangan koordinat (x, y)
    using Point = std::pair<double, double>;

    // Tahapan kerja program
    enum class Stage
    {
        KILL,             // Menghapus turtle lama
        SET_BACKGROUND,   // Mengubah background menjadi hitam
        SPAWN,            // Membuat turtle baru
        SET_PEN,          // Mengatur pena menjadi kuning
        CLEAR,             // Membersihkan layar pada awal program
        DRAW               // Menggambar pola
    };

    // ==========================================
    // OBJEK KOMUNIKASI ROS 2
    // ==========================================

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;

    rclcpp::Client<
        rcl_interfaces::srv::SetParameters>::SharedPtr background_client_;

    // Timer yang menjalankan state machine secara berkala
    rclcpp::TimerBase::SharedPtr timer_;

    // ==========================================
    // DATA POSISI TURTLE
    // ==========================================

    turtlesim::msg::Pose pose_;

    // Menandakan apakah data posisi sudah diterima
    bool pose_received_ = false;

    // ==========================================
    // PARAMETER MATEMATIS SPIROGRAPH
    // ==========================================

    double center_x_;
    double center_y_;

    double R_;
    double r_;
    double d_;

    int total_layers_;
    double total_rotation_deg_;

    // Indeks lapisan yang sedang digambar, dimulai dari nol
    int current_layer_ = 0;

    // Daftar sudut rotasi dan titik-titik jalur
    std::vector<double> rotations_;
    std::vector<Point> path_;

    // Indeks target jalur yang sedang diikuti
    std::size_t target_index_ = 0;

    // ==========================================
    // STATUS STATE MACHINE
    // ==========================================

    Stage stage_ = Stage::KILL;

    // Mencegah pengiriman beberapa permintaan service sekaligus
    bool operation_pending_ = false;

    // Memastikan layar hanya dibersihkan satu kali
    bool clear_done_ = false;

    // Menandakan background sudah berhasil diatur
    bool background_done_ = false;

    // Menandakan pengukuran waktu lapisan telah dimulai
    bool timing_started_ = false;

    // Waktu mulai menggambar satu lapisan
    std::chrono::steady_clock::time_point layer_start_time_;

    // Konstanta pi untuk perhitungan sudut
    static constexpr double PI = 3.14159265358979323846;

    // Mengubah sudut agar berada dalam rentang -pi sampai pi
    // Berguna untuk mendapatkan selisih sudut belok terpendek
    double normalize_angle(double angle)
    {
        while (angle > PI)
            angle -= 2.0 * PI;

        while (angle < -PI)
            angle += 2.0 * PI;

        return angle;
    }

    // ==========================================
    // MEMBUAT SUDUT ROTASI SETIAP LAPISAN
    // ==========================================

    void generate_rotations()
    {
        rotations_.clear();

        for (int i = 0; i < total_layers_; ++i)
        {
            // Membagi total sudut rotasi secara merata
            const double angle_deg =
                i * total_rotation_deg_ / total_layers_;

            // Mengubah derajat menjadi radian
            rotations_.push_back(angle_deg * PI / 180.0);
        }
    }

    // ==========================================
    // MENGHASILKAN JALUR SPIROGRAPH
    // ==========================================

    void generate_path()
    {
        path_.clear();

        // Jumlah pembagian dalam satu putaran penuh
        constexpr int NUM_POINTS = 720;

        // Sudut rotasi untuk lapisan yang sedang digambar
        const double rotation = rotations_[current_layer_];

        const double cr = std::cos(rotation);
        const double sr = std::sin(rotation);

        // Menghitung koordinat kurva dari t = 0 sampai 2*pi
        for (int i = 0; i <= NUM_POINTS; ++i)
        {
            const double t = 2.0 * PI * i / NUM_POINTS;

            // Persamaan kurva hypotrochoid
            const double x =
                center_x_
                + (R_ - r_) * std::cos(t)
                + d_ * std::cos(((R_ - r_) / r_) * t);

            const double y =
                center_y_
                + (R_ - r_) * std::sin(t)
                - d_ * std::sin(((R_ - r_) / r_) * t);

            // Menggeser koordinat relatif terhadap pusat pola
            const double dx = x - center_x_;
            const double dy = y - center_y_;

            // Memutar koordinat menggunakan matriks rotasi 2D
            const double rotated_x =
                center_x_ + dx * cr - dy * sr;

            const double rotated_y =
                center_y_ + dx * sr + dy * cr;

            // Simpan titik koordinat ke dalam daftar jalur
            path_.emplace_back(rotated_x, rotated_y);
        }

        // Mulai mengikuti jalur dari titik pertama
        target_index_ = 0;

        RCLCPP_INFO(
            get_logger(),
            "Jalur lapisan %d dibuat. Rotasi: %.1f derajat",
            current_layer_ + 1,
            rotations_[current_layer_] * 180.0 / PI
        );
    }

    // ==========================================
    // MENERIMA POSISI TURTLE
    // ==========================================

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        // Simpan posisi dan orientasi terbaru dari TurtleSim
        pose_ = *msg;
        pose_received_ = true;
    }

    // Mengirim perintah agar turtle berhenti
    void stop_turtle()
    {
        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;

        cmd_pub_->publish(cmd);
    }

    // ==========================================
    // MENGUBAH BACKGROUND MENJADI HITAM
    // ==========================================

    void request_background()
    {
        // Jangan mengirim permintaan baru jika masih ada yang berjalan
        if (operation_pending_)
            return;

        // Tunggu sampai service parameter tersedia
        if (!background_client_->service_is_ready())
            return;

        auto request =
            std::make_shared<
                rcl_interfaces::srv::SetParameters::Request>();

        // Helper untuk membuat parameter bertipe integer
        auto make_integer_parameter =
            [](const std::string &name, int value)
            {
                rcl_interfaces::msg::Parameter parameter;
                parameter.name = name;

                parameter.value.type =
                    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;

                parameter.value.integer_value = value;

                return parameter;
            };

        // Nilai RGB nol menghasilkan warna hitam
        request->parameters = {
            make_integer_parameter("background_r", 0),
            make_integer_parameter("background_g", 0),
            make_integer_parameter("background_b", 0)
        };

        operation_pending_ = true;

        // Kirim permintaan secara asynchronous
        background_client_->async_send_request(
            request,
            [this](
                rclcpp::Client<
                    rcl_interfaces::srv::SetParameters>::SharedFuture future)
            {
                operation_pending_ = false;

                try
                {
                    const auto response = future.get();
                    bool success = true;

                    // Periksa hasil setiap parameter
                    for (const auto &result : response->results)
                    {
                        if (!result.successful)
                        {
                            success = false;

                            RCLCPP_ERROR(
                                get_logger(),
                                "Parameter background gagal: %s",
                                result.reason.c_str()
                            );
                        }
                    }

                    if (success)
                    {
                        background_done_ = true;

                        RCLCPP_INFO(
                            get_logger(),
                            "Background berhasil diubah menjadi hitam."
                        );

                        // Lanjutkan ke pembuatan turtle
                        stage_ = Stage::SPAWN;
                    }
                    else
                    {
                        // Coba kembali jika parameter ditolak
                        stage_ = Stage::SET_BACKGROUND;
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Gagal mengatur background: %s",
                        e.what()
                    );

                    stage_ = Stage::SET_BACKGROUND;
                }
            }
        );
    }

    // ==========================================
    // MENGATUR PENA MENJADI KUNING
    // ==========================================

    void request_set_pen()
    {
        if (operation_pending_)
            return;

        if (!pen_client_->service_is_ready())
            return;

        auto request =
            std::make_shared<turtlesim::srv::SetPen::Request>();

        // Warna RGB: merah 255, hijau 255, biru 0 = kuning
        request->r = 255;
        request->g = 255;
        request->b = 0;

        // Ketebalan garis
        request->width = 3;

        // off = 0 berarti pena aktif
        request->off = 0;

        operation_pending_ = true;

        pen_client_->async_send_request(
            request,
            [this](
                rclcpp::Client<
                    turtlesim::srv::SetPen>::SharedFuture future)
            {
                operation_pending_ = false;

                try
                {
                    // Memeriksa respons service
                    future.get();

                    RCLCPP_INFO(
                        get_logger(),
                        "Warna pena diatur menjadi kuning."
                    );

                    // Bersihkan layar hanya pada awal program
                    stage_ = clear_done_ ? Stage::DRAW : Stage::CLEAR;
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Gagal mengatur warna pena: %s",
                        e.what()
                    );

                    // Ulangi permintaan jika gagal
                    stage_ = Stage::SET_PEN;
                }
            }
        );
    }

    // ==========================================
    // MENGHAPUS TURTLE LAMA
    // ==========================================

    void request_kill()
    {
        if (operation_pending_)
            return;

        if (!kill_client_->service_is_ready())
            return;

        auto request =
            std::make_shared<turtlesim::srv::Kill::Request>();

        request->name = "turtle1";

        operation_pending_ = true;

        kill_client_->async_send_request(
            request,
            [this](
                rclcpp::Client<
                    turtlesim::srv::Kill>::SharedFuture future)
            {
                operation_pending_ = false;

                try
                {
                    future.get();

                    RCLCPP_INFO(
                        get_logger(),
                        "Turtle sebelumnya berhasil dihapus."
                    );
                }
                catch (const std::exception &)
                {
                    // Tetap lanjut jika turtle lama memang tidak ada
                    RCLCPP_INFO(
                        get_logger(),
                        "Turtle lama tidak ditemukan; melanjutkan."
                    );
                }

                // Atur background sebelum membuat turtle
                stage_ = background_done_
                    ? Stage::SPAWN
                    : Stage::SET_BACKGROUND;
            }
        );
    }

    // ==========================================
    // MEMBUAT TURTLE PADA TITIK AWAL
    // ==========================================

    void request_spawn()
    {
        if (operation_pending_)
            return;

        if (!spawn_client_->service_is_ready())
            return;

        auto request =
            std::make_shared<turtlesim::srv::Spawn::Request>();

        // Posisi awal turtle diambil dari titik pertama jalur
        request->x = static_cast<float>(path_.front().first);
        request->y = static_cast<float>(path_.front().second);

        // Orientasi awal turtle
        // Nilai ini mengikuti arah singgung awal kurva untuk
        // parameter spirograph yang digunakan dalam program ini
        request->theta = static_cast<float>(
            -PI / 2.0 + rotations_[current_layer_]
        );

        request->name = "turtle1";

        operation_pending_ = true;

        spawn_client_->async_send_request(
            request,
            [this](
                rclcpp::Client<
                    turtlesim::srv::Spawn>::SharedFuture future)
            {
                operation_pending_ = false;

                try
                {
                    future.get();

                    // Tunggu pose baru dari turtle yang baru dibuat
                    pose_received_ = false;

                    target_index_ = 0;
                    timing_started_ = false;

                    RCLCPP_INFO(
                        get_logger(),
                        "Turtle lapisan %d berhasil dibuat.",
                        current_layer_ + 1
                    );

                    // Atur pena sebelum mulai menggambar
                    stage_ = Stage::SET_PEN;
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Gagal membuat turtle: %s",
                        e.what()
                    );

                    // Coba membuat turtle kembali
                    stage_ = Stage::SPAWN;
                }
            }
        );
    }

    // ==========================================
    // MEMBERSIHKAN LAYAR 
    // ==========================================

    void request_clear()
    {
        if (operation_pending_)
            return;

        if (!clear_client_->service_is_ready())
            return;

        auto request =
            std::make_shared<std_srvs::srv::Empty::Request>();

        operation_pending_ = true;

        clear_client_->async_send_request(
            request,
            [this](
                rclcpp::Client<
                    std_srvs::srv::Empty>::SharedFuture future)
            {
                operation_pending_ = false;

                try
                {
                    future.get();

                    clear_done_ = true;

                    RCLCPP_INFO(
                        get_logger(),
                        "Layar dibersihkan."
                    );

                    // Mulai menggambar setelah persiapan selesai
                    stage_ = Stage::DRAW;
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Gagal membersihkan layar: %s",
                        e.what()
                    );

                    stage_ = Stage::CLEAR;
                }
            }
        );
    }

    // ==========================================
    // MENYELESAIKAN SATU LAPISAN
    // ==========================================

    void finish_layer()
    {
        // Hentikan gerakan turtle
        stop_turtle();

        // Hitung durasi menggambar lapisan
        const auto end_time = std::chrono::steady_clock::now();

        const double elapsed_seconds =
            std::chrono::duration<double>(
                end_time - layer_start_time_
            ).count();

        RCLCPP_INFO(
            get_logger(),
            "Lapisan %d selesai dalam %.2f detik.",
            current_layer_ + 1,
            elapsed_seconds
        );

        // Pindah ke indeks lapisan berikutnya
        ++current_layer_;

        // Jika semua lapisan selesai, hentikan timer
        if (current_layer_ >= total_layers_)
        {
            RCLCPP_INFO(
                get_logger(),
                "Semua %d lapisan spirograph selesai!",
                total_layers_
            );

            timer_->cancel();
            return;
        }

        // Hitung jalur untuk lapisan berikutnya
        generate_path();

        pose_received_ = false;
        timing_started_ = false;

        // Hapus turtle sebelumnya sebelum membuat turtle baru.
        // Jejak gambar sebelumnya tetap dipertahankan.
        stage_ = Stage::KILL;
    }

    // ==========================================
    // KONTROL GERAK 
    // ==========================================

    void draw_path()
    {
        // Jangan bergerak sebelum pose diterima
        if (!pose_received_ || path_.empty())
        {
            stop_turtle();
            return;
        }

        // Catat waktu ketika lapisan mulai digambar
        if (!timing_started_)
        {
            layer_start_time_ = std::chrono::steady_clock::now();
            timing_started_ = true;
        }

        const std::size_t last_index = path_.size() - 1;

        // Batasi pencarian ke sejumlah titik di depan
        // agar tidak mudah melompat ke cabang kurva lain
        constexpr std::size_t SEARCH_AHEAD = 12;

        const std::size_t search_end =
            std::min(target_index_ + SEARCH_AHEAD, last_index);

        // Cari titik terdekat dalam jendela pencarian
        std::size_t nearest_index = target_index_;

        double nearest_distance =
            std::numeric_limits<double>::max();

        for (std::size_t i = target_index_; i <= search_end; ++i)
        {
            const double dx = path_[i].first - pose_.x;
            const double dy = path_[i].second - pose_.y;

            const double distance = std::hypot(dx, dy);

            if (distance < nearest_distance)
            {
                nearest_distance = distance;
                nearest_index = i;
            }
        }

        // Perbarui progres jalur
        target_index_ = nearest_index;

        // Periksa jarak ke titik terakhir
        const double end_distance = std::hypot(
            path_.back().first - pose_.x,
            path_.back().second - pose_.y
        );

        // Lapisan selesai jika indeks telah mencapai ujung
        // dan posisi turtle cukup dekat dengan titik terakhir
        if (target_index_ >= last_index && end_distance < 0.15)
        {
            finish_layer();
            return;
        }

        // Tentukan titik target berdasarkan jarak sepanjang jalur
        constexpr double LOOKAHEAD_DISTANCE = 0.20;

        std::size_t target = target_index_;
        double path_distance = 0.0;

        while (target < last_index &&
               path_distance < LOOKAHEAD_DISTANCE)
        {
            const double dx =
                path_[target + 1].first - path_[target].first;

            const double dy =
                path_[target + 1].second - path_[target].second;

            // Akumulasi jarak antar-titik pada jalur
            path_distance += std::hypot(dx, dy);

            ++target;
        }

        // Hitung selisih posisi turtle terhadap titik target
        const double dx = path_[target].first - pose_.x;
        const double dy = path_[target].second - pose_.y;

        const double target_distance = std::hypot(dx, dy);

        // Sudut arah dari turtle menuju target
        const double target_angle = std::atan2(dy, dx);

        // Selisih arah target dengan orientasi turtle
        const double alpha =
            normalize_angle(target_angle - pose_.theta);

        // Kurangi kecepatan maju saat perlu berbelok tajam
        double linear_speed = 0.20;

        if (std::abs(alpha) > 1.0)
            linear_speed = 0.08;
        else if (std::abs(alpha) > 0.65)
            linear_speed = 0.12;

        // Hindari pembagian dengan jarak yang terlalu kecil
        const double safe_distance =
            std::max(target_distance, 0.05);

        // Rumus Pure Pursuit untuk menentukan kecepatan sudut
        // omega = 2 * v * sin(alpha) / jarak target
        const double angular_speed =
            2.0 * linear_speed * std::sin(alpha) / safe_distance;

        geometry_msgs::msg::Twist cmd;

        // Kirim perintah gerak ke TurtleSim
        cmd.linear.x = linear_speed;

        // Batasi kecepatan putar agar gerakan tetap terkendali
        cmd.angular.z = std::clamp(
            angular_speed,
            -1.5,
            1.5
        );

        cmd_pub_->publish(cmd);
    }

    // ==========================================
    // STATE MACHINE UTAMA
    // ==========================================

    void control_loop()
    {
        switch (stage_)
        {
            case Stage::KILL:
                stop_turtle();
                request_kill();
                break;

            case Stage::SET_BACKGROUND:
                request_background();
                break;

            case Stage::SPAWN:
                request_spawn();
                break;

            case Stage::SET_PEN:
                request_set_pen();
                break;

            case Stage::CLEAR:
                request_clear();
                break;

            case Stage::DRAW:
                draw_path();
                break;
        }
    }
};

int main(int argc, char **argv)
{
    // Inisialisasi ROS 2
    rclcpp::init(argc, argv);

    // Buat objek node Spirograph
    auto node = std::make_shared<Spirograph>();

    // Jalankan node agar callback dan timer diproses
    rclcpp::spin(node);

    // Matikan ROS 2 setelah node berhenti
    rclcpp::shutdown();

    return 0;
}
