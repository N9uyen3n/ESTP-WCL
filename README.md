
## 📂 Giải thích chi tiết

| File / Thư mục      | Vai trò chính                                                 |
|---------------------|---------------------------------------------------------------|
| `Node.*`            | Mô tả các node giữa các node trong bài toán                   |
| `Arc.*`             | Mô tả các cạnh/cung giữa các node trong bài toán              |
| `ChargingOption.*`  | Mô tả thông tin trạm sạc và tùy chọn sạc                      |
| `CSVReader.*`       | Đọc dữ liệu từ các file CSV như `nodes.csv`, `params.csv`,... |
| `LocalSearch1.cpp`  | Chứa thuật toán tìm kiếm cục bộ (Local Search / VNS)          |
| `Node.*`            | Mô tả các điểm (node) trong hệ thống                          |
| `Optimizer.*`       | Thuật toán tối ưu chính (MILP hoặc heuristic)                 |
| `Params.*`          | Xử lý tham số bài toán từ file cấu hình                       |
| `TestOptimizer.cpp` | Chạy thử nghiệm các thuật toán với bộ dữ liệu đã cho          |
| `TestOptimizer.obj` | File biên dịch trung gian (có thể bỏ qua)                     |
| `README.md`         | Tài liệu mô tả cấu trúc dự án và cách sử dụng                 |

---

## 📌 Ghi chú
- Thư mục `data/Input/` chứa các file như:
    - `charging_options.csv`, `nodes.csv`, `params.csv`, `wireless_arcs.csv`
- Kết quả đầu ra được ghi vào `data/Output/`

---

> ✨ Nếu bạn cần hướng dẫn chạy chương trình, build bằng CMake, hay ví dụ test cụ thể, mình có thể thêm vào phần tiếp theo trong README nhé!
