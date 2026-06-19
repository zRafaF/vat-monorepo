import random

def define_env(env):
    @env.macro
    def lorem(paragraphs=1):
        # We create a private RNG instance with a fixed seed (e.g., 42)
        # This ensures the output is identical on every server reload.
        rng = random.Random(42)
        
        words = [
            "lorem", "ipsum", "dolor", "sit", "amet", "consectetur", "adipiscing", 
            "elit", "sed", "do", "eiusmod", "tempor", "incididunt", "ut", "labore", 
            "et", "dolore", "magna", "aliqua", "ut", "enim", "ad", "minim", "veniam", 
            "quis", "nostrud", "exercitation", "ullamco", "laboris", "nisi", "ut", 
            "aliquip", "ex", "ea", "commodo", "consequat", "duis", "aute", "irure", 
            "dolor", "in", "reprehenderit", "in", "voluptate", "velit", "esse", 
            "cillum", "dolore", "eu", "fugiat", "nulla", "pariatur", "excepteur", 
            "sint", "occaecat", "cupidatat", "non", "proident", "sunt", "in", 
            "culpa", "qui", "officia", "deserunt", "mollit", "anim", "id", "est", "laborum"
        ]

        def generate_sentence():
            # Use rng.randint and rng.choice instead of the global random module
            sentence_length = rng.randint(8, 15)
            sentence_words = [rng.choice(words) for _ in range(sentence_length)]
            return " ".join(sentence_words).capitalize() + "."

        def generate_paragraph():
            num_sentences = rng.randint(4, 7)
            return " ".join([generate_sentence() for _ in range(num_sentences)])

        result = [generate_paragraph() for _ in range(paragraphs)]
        return "\n\n".join(result)