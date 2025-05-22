import nltk, re
nltk.download('wordnet')
from nltk.corpus import wordnet

words = """
car-automobile, gem-jewel, journey-voyage, boy-lad, coast-shore, asylum-madhouse, magician-
wizard, midday-noon, furnace- stove, food-fruit, bird-cock, bird-crane, tool-implement, brother-
monk, lad- brother, crane-implement, journey-car, monk-oracle, cemetery-woodland, food-
rooster, coast-hill, forest-graveyard, shore-woodland, monk-slave, coast-forest, lad-wizard,
chord-smile, glass-magician, rooster-voyage, noon-string
"""

pairs = [tuple(p.split('-')) for p in re.sub(r'\s+', '', words).split(',')] # [tuple(pair.replace("\n", "").replace(", ", ",").replace(" ", "").split("-")) for pair in words.strip().split(",")]

def get_similarity_scores(pairs):
    scores = []
    for w1, w2 in pairs:
        synsets1 = wordnet.synsets(w1, pos=wordnet.NOUN)
        synsets2 = wordnet.synsets(w2, pos=wordnet.NOUN)
        max_score = 0
        for s1 in synsets1:
            for s2 in synsets2:
                sim = s1.path_similarity(s2)
                if sim and sim > max_score:
                    max_score = sim
        scores.append(((w1, w2), max_score))
    return sorted(scores, key=lambda x: (x[1] if x[1] is not None else -1), reverse=True)

if __name__ == "__main__":
    similarities = get_similarity_scores(pairs)
    for pair, score in similarities:
        print(f"{pair[0]} - {pair[1]}: {score:.3f}")
