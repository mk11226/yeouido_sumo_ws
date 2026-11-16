"""여의도 환경 전체 생성: 다운로드 → 도로망 → 교통량 → 지점 연결 → 수요 → 주변 환경·랜드마크 → 그림 → 검증.

    python -m yeouido.run_all                 # 처음부터 끝까지 (검증 포함, 약 40분)
    python -m yeouido.run_all --skip-validate # 검증 시뮬레이션 생략 (약 1분)

이미 받은 원본 파일(env/yeouido/source/)은 다시 받지 않는다.
"""
import argparse

from yeouido import (build_context, build_demand, build_net, download, landmarks,
                     map_counts, nodelink, plot, prepare_counts, validate)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--skip-validate", action="store_true")
    args = ap.parse_args()
    print("── 1. 원본 다운로드 (TOPIS 교통량 엑셀, OSM 신호 위치·주변 환경)")
    download.main([])
    print("── 2. 국가표준노드링크에서 범위 추출 → SUMO 도로망 생성")
    if (nodelink.RAW / "MOCT_LINK.shp").exists() or not nodelink.REGION.exists():
        nodelink.main()          # 원본이 있으면 다시 추출
    else:
        print("   노드링크 원본 없음 → 이미 추출한 region.json 사용")
    build_net.main([])
    print("── 3. TOPIS 교통량 정리")
    prepare_counts.main()
    print("── 4. 측정 지점 → 도로 연결, 게이트 A~F")
    map_counts.main()
    print("── 5. 시간대별 일반 차량 수요")
    build_demand.main()
    print("── 주변 환경(건물·공원·한강)과 목적지 장소")
    build_context.main()
    landmarks.main()
    plot.main()
    if not args.skip_validate:
        print("── 6. 시뮬레이션 검증")
        validate.main([])
    print("완료. 화면으로 확인: python -m yeouido.view --level rush")


if __name__ == "__main__":
    main()
