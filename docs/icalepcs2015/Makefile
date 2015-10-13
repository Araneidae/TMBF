# Need to run:
#       module load texlive/2015
#
PATH := /dls_sw/apps/texlive/2015/x86_64-linux/:$(PATH)

LATEX = pdflatex

DOCUMENT = MOPGF097

DEPENDS += $(wildcard *.tikz)

# Seven figures need to be convered for the poster
POSTER_DEPENDS += $(foreach n,1 2 3 4 5 6 7,figure$(n).pdf)
POSTER_DEPENDS += diamond_logo.eps diamond-background.png

TARGETS = $(DOCUMENT).pdf $(DOCUMENT)_poster.pdf


default: $(TARGETS)

CITE_GREP = There were undefined references.
REF_GREP = Rerun to get cross-references right.
NOFILE_GREP = ^No file

%.pdf: %.tex
	$(LATEX) </dev/null -halt-on-error $*
	if grep -q '$(CITE_GREP)\|$(REF_GREP)\|$(NOFILE_GREP)' $*.log; then \
            $(LATEX) </dev/null -halt-on-error $*; \
        fi
	rm -f $*.log

# Extract each figure as a separate page
# I am told that pdftk treats embedded images better, but we don't have it.
figure%.pdf: make_figures.pdf
	gs -sDEVICE=pdfwrite -dNOPAUSE -dBATCH -dSAFER \
            -dFirstPage=$* -dLastPage=$* \
            -sOutputFile=$@ $<

clean:
	rm -f *.aux *.log
	rm -f $(filter-out $(TARGETS),$(wildcard *.pdf))

clean-all: clean
	rm -f $(TARGETS)


$(DOCUMENT).pdf: $(DEPENDS)

make_figures.pdf: $(DEPENDS)

$(DOCUMENT)_poster.pdf: $(POSTER_DEPENDS)


.PHONY: default clean clean-all
