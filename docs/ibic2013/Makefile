DOCUMENT = WEPC10

FIGURES += WEPC10f1.eps
FIGURES += WEPC10f2.eps


default: $(DOCUMENT).pdf

$(DOCUMENT).pdf: $(FIGURES:.eps=.pdf)

CITE_GREP = There were undefined references.
REF_GREP = Rerun to get cross-references right.
NOFILE_GREP = ^No file

LATEX = pdflatex

%.pdf: %.tex
	$(LATEX) </dev/null -halt-on-error $*
	if grep -q '$(CITE_GREP)\|$(REF_GREP)\|$(NOFILE_GREP)' $*.log; then \
            $(LATEX) </dev/null -halt-on-error $*; \
        fi
	rm -f $*.log

%.pdf: %.eps
	epstopdf $^

clean:
	rm -f $(DOCUMENT).aux

clean-all: clean
	rm -f *.pdf

.PHONY: default clean clean-all
