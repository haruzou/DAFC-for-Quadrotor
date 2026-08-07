# Direct Adaptive Fuzzy Controller-for-Quadrotors
Thuật toán điều khiển chính ở đây là Mờ thích nghi (Adaptive fuzzy), thay vì dùng mệnh đề mờ IF...THEN... như thông thường
thì tác giả đã sử dụng các hàm toán học để giải TRỰC TIẾP đầu ra từ Hệ suy luận mờ

## Luật Mờ
Trong điều khiển mờ truyền thống:

    R1: IF $X_1$ is $A_{1,1}$ AND $X_2$ is $A_{2,1}$ THEN $\gamma$ is $B_1$ OR
    R2: IF $X_1$ is $A_{1,2}$ AND $X_2$ is $A_{2,2}$ THEN $\gamma$ is $B_2$

Trong bài, luật mờ đã được nhúng chìm vào tích Kronecker. Các hàm zeta_p và zeta_a chính là bước tính độ thỏa mãn của TẤT CẢ các luật IF...THEN cùng một lúc:

Vòng lặp 6 x 6 = 36 này tương đương với việc khai báo 36 luật Mờ IF-THEN.

## THEN
Phần THEN tronmg bài không cố đinh mà được thay đổi tự động. Nó được cập nhật thời gian thực bằng thuật toán thích nghi Adaptive dựa trên sai số điều khiển:

    Theta_x_dot = gamma_p * (ex*pnp(1) + ex_dot*pnp(2)) * zeta_x;

## Giải mờ
FIS trong bài sử dụng mô hình mờ Takagi-Sugeno (Zero-order TS Fuzzy System) hoặc Singleton Defuzzification. Lượng đầu ra tín hiệu điều khiển được tính đơn giản bằng tích vô hướng giữa trọng số $\Theta$ và độ thỏa mãn luật $\zeta$:

    ux = Theta_x.' * zeta_x;
## Kết quả mô phỏng ( trường hợp không có nhiễu)
### Simulink
