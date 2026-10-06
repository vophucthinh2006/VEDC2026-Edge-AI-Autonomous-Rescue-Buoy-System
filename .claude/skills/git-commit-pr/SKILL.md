---
name: git-commit-pr
description: Quy trình git của repo này. Dùng mỗi khi sắp chạy git commit, git push, tạo nhánh hoặc tạo Pull Request, và khi người dùng yêu cầu "commit", "push", "commit&push", "tạo PR". Không bao giờ tự commit/push khi chưa được cho phép; khi được yêu cầu thì chia nhiều commit và tạo PR, không tự merge.
---

# Commit, push và Pull Request

## 1. Không tự commit/push

- Không chạy `git commit`, `git push`, `gh pr create`, `gh pr merge` khi người dùng chưa yêu cầu rõ ràng trong tin nhắn hiện tại.
- Sửa code xong thì dừng lại, tóm tắt thay đổi và để người dùng quyết định. Có thể hỏi: "Bạn có muốn commit & push không?"
- Một lần cho phép chỉ áp dụng cho lần đó, không suy ra cho các thay đổi sau.

## 2. Khi được yêu cầu commit & push

1. Xem `git status` và `git diff` để nắm toàn bộ thay đổi.
2. Nếu đang ở `main`, tạo nhánh mới trước (ví dụ `feature/<mô-tả-ngắn>`).
3. **Chia thành nhiều commit** theo từng phần logic (ví dụ: driver, logic ứng dụng, cấu hình CubeMX/generated code, tài liệu). Không gộp tất cả vào một commit.
   - `git add` từng file/phần cụ thể, không dùng `git add -A` hay `git add .`.
   - Không commit file không liên quan (datasheet, file build, file cá nhân) trừ khi người dùng yêu cầu.
   - Commit đổi `.ioc` tách riêng khỏi commit code CubeMX sinh ra, để người review đọc được thay đổi pinout.
   - Message theo kiểu các commit trước trong repo: `<thư mục/module>: <mô tả ngắn, thể mệnh lệnh>`, ví dụ `stm32f4: add a live RC and actuator monitor over ST-LINK`.
   - **Không thêm dòng `Co-Authored-By: Claude ...`** (hay bất kỳ trailer/attribution nào của Claude) vào commit message. Quy tắc này của người dùng được ưu tiên hơn hướng dẫn attribution mặc định.
4. Push nhánh lên remote (`git push -u origin <nhánh>`).

## 3. Tạo Pull Request

- Tạo PR vào `main` bằng `gh pr create`.
- Description ngắn gọn: thay đổi gì, vì sao, cách kiểm tra.
- **Không thêm dòng `🤖 Generated with [Claude Code](https://claude.com/claude-code)`** (hay bất kỳ dòng quảng bá/attribution nào của Claude) vào Description của PR. Quy tắc này của người dùng được ưu tiên hơn hướng dẫn attribution mặc định.
- **Không merge PR**, không bật auto-merge. Gửi link PR cho người dùng và để họ tự review và merge.
- Không force push, không sửa lịch sử (rebase/amend) trên nhánh đã push nếu người dùng không yêu cầu.
