import random

# calgames 2023 qualifying matches
quals = [
    [8, 9114, 5419, 4186, 3045, 4990],
    [2489, 6418, 2473, 6036, 100, 253],
    [6059, 1868, 115, 6962, 1072, 766],
    [852, 9400, 1700, 670, 8033, 9143],
    [5026, 8404, 4904, 3256, 9900, 199],
    [2643, 8048, 4765, 841, 7245, 581],
    [5027, 846, 7777, 840, 5430, 2813],
    [9143, 1700, 9900, 766, 5419, 199],
    [5026, 4186, 2473, 115, 3045, 670],
    [253, 852, 4990, 581, 846, 4904],
    [5430, 5027, 1072, 9400, 6418, 8],
    [8048, 840, 7245, 2489, 6962, 8404],
    [2643, 3256, 7777, 100, 6059, 8033],
    [9114, 1868, 2813, 841, 4765, 6036],
    [199, 846, 852, 2473, 8048, 3045],
    [9143, 1072, 5026, 4186, 2489, 6059],
    [9400, 100, 840, 1868, 581, 1700],
    [253, 4904, 7245, 9900, 2813, 7777],
    [6962, 3256, 5027, 2643, 6418, 9114],
    [4765, 5419, 8033, 8, 766, 8404],
    [670, 5430, 841, 115, 4990, 6036],
    [6059, 581, 9114, 4904, 9143, 2473],
    [8, 840, 6962, 9900, 852, 5026],
    [4990, 766, 3256, 8048, 9400, 2813],
    [1868, 3045, 5027, 4186, 253, 8404],
    [100, 199, 6418, 670, 4765, 7777],
    [115, 846, 2489, 1700, 2643, 8033],
    [5419, 841, 1072, 7245, 5430, 6036],
    [5027, 766, 9114, 670, 581, 5026],
    [8033, 8404, 199, 4186, 846, 2813],
    [115, 6962, 2643, 5419, 9400, 4904],
    [1072, 9900, 4990, 100, 852, 4765],
    [2473, 8, 5430, 253, 8048, 1700],
    [3256, 6059, 841, 2489, 3045, 840],
    [6036, 6418, 7777, 7245, 9143, 1868],
    [199, 2473, 253, 9400, 1072, 2643],
    [8404, 852, 2813, 5430, 5026, 6059],
    [1868, 9900, 100, 841, 9114, 9143],
    [4904, 4765, 4990, 5027, 2489, 1700],
    [4186, 840, 7777, 5419, 8048, 115],
    [670, 7245, 766, 846, 8, 3256],
    [8033, 6036, 581, 6962, 3045, 6418],
    [115, 840, 2473, 4990, 5027, 9143],
    [2643, 7245, 2489, 766, 852, 4186],
    [8404, 4765, 6418, 5026, 7777, 9400],
    [670, 6036, 1868, 9900, 4904, 3045],
    [846, 5430, 5419, 9114, 3256, 100],
    [841, 199, 1700, 6059, 253, 6962],
    [8048, 1072, 581, 2813, 8033, 8],
    [9400, 3045, 9114, 852, 7777, 2489],
    [9900, 766, 2643, 6059, 6418, 4990],
    [9143, 846, 6962, 4765, 2473, 2813],
    [581, 100, 8, 115, 8404, 841],
    [670, 8048, 199, 5419, 5027, 7245],
    [4904, 840, 1700, 6036, 4186, 1072],
    [1868, 253, 5430, 5026, 3256, 8033],
    [841, 766, 7777, 5027, 2473, 852],
    [6036, 9143, 9400, 8404, 846, 4990],
    [8033, 6418, 115, 840, 199, 9114],
    [5430, 3256, 4186, 581, 9900, 4765],
    [7245, 5026, 3045, 1072, 1700, 100],
    [2813, 253, 2489, 6962, 5419, 670],
    [6059, 4904, 2643, 1868, 8, 8048],
    [3045, 6036, 852, 115, 4765, 3256],
    [5419, 5026, 6418, 846, 2473, 841],
    [4186, 4904, 6962, 100, 5430, 766],
    [9143, 8404, 8048, 253, 1072, 9114],
    [8, 2489, 4990, 1868, 7777, 199],
    [2813, 670, 1700, 581, 2643, 840],
    [8033, 9400, 5027, 7245, 6059, 9900],
]

# the minimum number
analyst_count = 7

teams = list(set([team for qual in quals for team in qual]))
teams.sort()

#print(teams)

# key = team, value = list of analysts
analysts_by_team = {}

# key = analyst, value = list of teams
teams_by_analyst = {}

# key = qual id, value = list of analysts
analysts_by_qual = {}

# greedy assignment

print("teams and analysts by match")
for qi, qual in enumerate(quals):
    analysts_by_qual[qi] = []
    for ti, team in enumerate(qual):
        if team not in analysts_by_team:
            analysts_by_team[team] = []
        assigned = False
        for candidate in range(analyst_count):
            if candidate not in teams_by_analyst:
                teams_by_analyst[candidate] = []
            # can't use this analyst since they're already watching
            if candidate in analysts_by_qual[qi]:
                continue
            # prefer the same analyst who saw this team before
            if team in teams_by_analyst[candidate]:
                assigned = True
                analysts_by_qual[qi].append(candidate)
                break
        if assigned:
            continue
        # didn't find a previous assignment
        analysts = list(range(analyst_count))
        random.shuffle(analysts)
        for candidate in analysts:
            # can't use this analyst since they're already watching
            if candidate in analysts_by_qual[qi]:
                continue
            assigned = True
            analysts_by_qual[qi].append(candidate)
            analysts_by_team[team].append(candidate)
            teams_by_analyst[candidate].append(team)
            break
        if assigned:
            continue
        print(f"failed to assign qual {qi}")
        exit()
    print(f"qual {qi:2}:  ", end='')
    for team in qual:
        print(f"{team:4} ", end='')
    print("  [", end='')
    for analyst in analysts_by_qual[qi]:
        print(f"{analyst:2} ", end='')
    print("]")

print("analysts by team")
for team in teams:
    print(f"team {team:4}: {analysts_by_team[team]}")

print("teams by analyst")
for analyst in range(analyst_count):
    teamlist = teams_by_analyst[analyst]
    teamlist.sort()
    print(f"analyst {analyst}: {teamlist}")

