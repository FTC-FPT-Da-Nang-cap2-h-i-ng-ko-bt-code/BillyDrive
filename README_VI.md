<div align="center">
<a href="README.md">🇬🇧 English</a> |
<a href="README_VI.md">🇻🇳 Tiếng Việt</a>
</div>
# JavaOnRobot Pathing

> **Một hệ thống Pathing tối giản, dễ tích hợp và có độ chính xác cao dành cho JavaOnRobot.**

## 🚀 Giới thiệu

**JavaOnRobot Pathing** là một bộ thư viện Pathing được xây dựng dành cho các dự án robot sử dụng JavaOnRobot.

Mục tiêu của dự án là tạo ra một hệ thống **pathing mạnh nhưng không phức tạp**, giúp lập trình viên có thể nhanh chóng đưa hệ thống vào robot, dễ dàng tuning và đặc biệt là dễ sửa đổi khi robot thay đổi cơ cấu hoặc gặp lỗi.

Thay vì phải xây dựng một hệ thống Pathing phức tạp từ đầu, JavaOnRobot Pathing hướng tới việc chỉ cần **tải project → đưa vào robot → cấu hình → sử dụng**.

---

## ✨ Tính năng chính

### 📦 Dễ dàng tích hợp

Hệ thống được thiết kế để có thể **tải xuống và đưa trực tiếp vào robot** với lượng cấu hình tối thiểu.

Mục tiêu là giảm thời gian setup và giúp người lập trình tập trung vào việc xây dựng autonomous thay vì phải xử lý quá nhiều phần nền tảng.

---

### 📄 Một file hoặc nhiều file Pathing

JavaOnRobot Pathing hỗ trợ nhiều cách tổ chức code:

**Cấu hình đơn giản:**

```text
Pathing.java
```

Một file duy nhất có thể chứa toàn bộ hệ thống cần thiết.

**Hoặc tách riêng Path:**

```text
Pathing.java
Path1.java
Path2.java
Path3.java
...
```

Trong đó:

* `Pathing.java` → hệ thống điều khiển và tuning
* `Path*.java` → các đường đi/path riêng

Điều này giúp project vừa có thể **đơn giản hóa cho robot nhỏ**, vừa có thể **mở rộng cho những project lớn**.

---

## 🎯 PID tối giản nhưng chính xác

Một trong những mục tiêu quan trọng của dự án là xây dựng hệ thống PID:

* Ít thông số cần quan tâm.
* Dễ hiểu.
* Dễ tuning.
* Dễ thay đổi.
* Không yêu cầu một lượng lớn code.
* Vẫn đảm bảo khả năng đạt độ chính xác cao.

Thay vì khiến người dùng phải hiểu quá nhiều thành phần phức tạp, hệ thống hướng tới một API đơn giản, ví dụ:

```java
PID pid = new PID(kP, kI, kD);
```

và sau đó có thể sử dụng trực tiếp trong hệ thống Pathing.

> **Mục tiêu: ít code hơn, nhưng không đánh đổi độ chính xác.**

---

## 🛠️ Khả năng tùy biến cao

JavaOnRobot Pathing không cố gắng biến mọi thứ thành một hệ thống "đóng".

Các thành phần quan trọng được thiết kế để có thể dễ dàng:

* Thay đổi thuật toán.
* Thay đổi PID.
* Thay đổi cách tính velocity.
* Thay đổi cách robot di chuyển.
* Thay đổi cách xử lý target.
* Thêm sensor.
* Thêm hệ thống correction.
* Thay đổi cách debug.
* Viết lại từng hàm theo nhu cầu của robot.

Điều này đặc biệt hữu ích khi robot có cơ cấu đặc biệt hoặc cần một cách điều khiển khác với hệ thống mặc định.

---

## 🐛 Dễ dàng Debug và sửa lỗi

Code được định hướng theo tiêu chí:

> **Nếu có lỗi, phải dễ tìm ra lỗi nằm ở đâu.**

Các thành phần của hệ thống được tách biệt rõ ràng để việc debug không trở thành một "mê cung" code.

Thay vì phải sửa nhiều class hoặc phụ thuộc vào một framework lớn, lập trình viên có thể đi thẳng tới hàm cần sửa và thay đổi nó.

---

## 🧩 Kiến trúc hướng tới sự đơn giản

Hệ thống được xây dựng theo hướng:

```text
Robot
  │
  ▼
Pathing
  │
  ├── PID
  ├── Odometry / Localization
  ├── Motion Control
  └── Path
```

Các thành phần có thể được thay thế hoặc mở rộng tùy theo nhu cầu của robot.

---

## 🔮 Roadmap

### Hiện tại

🟡 **Đang phát triển**

Hiện tại dự án vẫn đang tập trung hoàn thiện **nền móng cơ bản của một hệ thống Pathing**.

Các thành phần nền tảng cần được xây dựng và kiểm tra kỹ trước khi tiến tới những phần thực sự đặc biệt của dự án.

---

### Tương lai

#### 🤖 Auto Tuning

Một trong những mục tiêu lớn của dự án là xây dựng hệ thống **Auto Tuning**.

Thay vì phải tự tìm `kP`, `kI`, `kD` bằng tay, người dùng có thể thực hiện quá trình tuning chỉ với **vài nút bấm**.

Mục tiêu hướng tới:

```text
Chọn chế độ
      ↓
Nhấn Tune
      ↓
Robot tự chạy test
      ↓
Thu thập dữ liệu
      ↓
Tính toán thông số
      ↓
Hoàn thành
```

Điều này sẽ giúp giảm đáng kể thời gian tuning robot.

---

## 📌 Trạng thái dự án

| Thành phần        | Trạng thái         |
| ----------------- | ------------------ |
| Core Pathing      | 🟡 Đang phát triển |
| PID               | 🟡 Đang phát triển |
| Motion Control    | 🟡 Đang phát triển |
| Path System       | 🟡 Đang phát triển |
| Debug System      | 🟡 Đang phát triển |
| Documentation     | 🔴 Chưa hoàn thiện |
| Auto Tuning       | ⚪ Planned          |
| Advanced Features | ⚪ Planned          |

---

## 💡 Triết lý của dự án

JavaOnRobot Pathing không hướng tới việc trở thành hệ thống có nhiều tính năng nhất.

Nó hướng tới việc trở thành một hệ thống:

**Đơn giản → Dễ dùng → Dễ sửa → Dễ mở rộng → Chính xác.**

Phần nền móng được ưu tiên hoàn thiện trước. Sau khi hệ thống Pathing cơ bản đủ ổn định, dự án sẽ tiếp tục phát triển những tính năng thực sự khác biệt thay vì chỉ thêm những tính năng phức tạp không cần thiết.

---

## 🚧 Project Status

> **JavaOnRobot Pathing is currently under active development.**

Nền móng của hệ thống đang được xây dựng và hoàn thiện. Các API và cấu trúc project hiện tại **có thể thay đổi** trong quá trình phát triển.

⭐ Nếu dự án hữu ích với bạn, hãy **Star repository** để theo dõi quá trình phát triển.
