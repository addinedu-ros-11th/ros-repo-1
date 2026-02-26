"""
명령(Command) vs 일반 챗(Greeting/General Question) 분기 테스트
2-Stage 의도 분류가 올바르게 동작하는지 확인
"""

import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ai_server.services.llm_service import LLMService


# ── 테스트 케이스 정의 ──────────────────────────────────────
# (입력 텍스트, 기대하는 카테고리)
#   카테고리: "CHAT" = GREETING 또는 GENERAL_QUESTION
#             "COMMAND" = SNACK_DELIVERY, ITEM_DELIVERY, GUIDE_GUEST 등 로봇 작업

TEST_CASES = [
    # ── 일반 챗: GREETING ──
    ("안녕하세요", "CHAT"),
    ("반가워", "CHAT"),
    ("좋은 아침이야", "CHAT"),
    ("수고해", "CHAT"),
    # ── 일반 챗: GENERAL_QUESTION ──
    ("오늘 날씨 어때?", "CHAT"),
    ("뭐 할 수 있어?", "CHAT"),
    ("고마워", "CHAT"),
    ("회사 규정 좀 알려줘", "CHAT"),
    # ── 명령: SNACK_DELIVERY ──
    ("회의실로 커피 2잔 갖다줘", "COMMAND"),
    ("초코파이 가져와줘", "COMMAND"),
    # ── 명령: ITEM_DELIVERY ──
    ("김철수한테 서류 전달해줘", "COMMAND"),
    # ── 명령: GUIDE_GUEST ──
    ("방문객을 3층 회의실로 안내해줘", "COMMAND"),
]


def run_test():
    print("\n" + "=" * 70)
    print("  [테스트] 명령(Command) vs 일반 챗(Chat) 분기 테스트")
    print("=" * 70)

    llm = LLMService()
    llm.initialize()
    print("LLM 서비스 초기화 완료\n")

    passed = 0
    failed = 0
    results_log = []

    for idx, (text, expected_category) in enumerate(TEST_CASES, 1):
        print(f'[{idx}/{len(TEST_CASES)}] 입력: "{text}"')
        start = time.time()

        result = llm.parse_natural_language(text)
        elapsed = time.time() - start

        task_type = result.get("task_type", "UNKNOWN")
        is_chat = result.get("is_chat", False)
        actual_category = "CHAT" if is_chat else "COMMAND"

        ok = actual_category == expected_category
        status = "✅ PASS" if ok else "❌ FAIL"

        if ok:
            passed += 1
        else:
            failed += 1

        # 결과 출력
        print(f"  task_type = {task_type}")
        print(f"  is_chat   = {is_chat}")
        if is_chat:
            chat_msg = result.get("fields", {}).get("message", "")
            print(f'  chat_reply= "{chat_msg[:80]}"')
        else:
            fields = result.get("fields", {})
            print(f"  fields    = {fields}")
        print(
            f"  기대: {expected_category} → 실제: {actual_category}  [{status}]  ({elapsed:.1f}s)"
        )
        print()

        results_log.append(
            {
                "text": text,
                "expected": expected_category,
                "actual": actual_category,
                "task_type": task_type,
                "ok": ok,
            }
        )

    # ── 요약 ──
    print("=" * 70)
    print(f"  결과: {passed} passed / {failed} failed / {len(TEST_CASES)} total")
    print("=" * 70)

    if failed:
        print("\n  실패한 케이스:")
        for r in results_log:
            if not r["ok"]:
                print(
                    f"    - \"{r['text']}\"  기대={r['expected']}  실제={r['actual']}  (task_type={r['task_type']})"
                )
    print()


if __name__ == "__main__":
    run_test()
