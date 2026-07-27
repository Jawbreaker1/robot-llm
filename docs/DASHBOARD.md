# Lokal Mac-dashboard

Dashboarden är en rörelsefri arbetsbänk för Robot LLM Lab. Den fungerar utan
EV3-batterier och samlar lokal dialog, read-only research, tekniska händelser,
agentbudgetar och ett beskrivande register över nuvarande och framtida
robotkomponenter.

## Starta

LM Studio är valfritt för att öppna gränssnittet, men krävs för att få svar
från Gemma.

```sh
PYTHONPATH=src python3 -m robot_agent.dashboard_cli
```

Öppna sedan den sessionsunika adress som skrivs ut, i formen
`http://127.0.0.1:8765/session/<session-token>/`. Länken är en lokal
bearer-hemlighet: dela den inte. En omstart myntar en ny länk och gör den
gamla ogiltig.

Servern binder alltid till den numeriska loopback-adressen `127.0.0.1`.
Porten kan ändras:

```sh
PYTHONPATH=src python3 -m robot_agent.dashboard_cli --port 8877
```

## Språk

Språkväljaren i toppfältet byter hela dashboardens presentationsspråk utan
omladdning eller ny nätverksbegäran. Svenska och engelska är kompletta
kataloger i den första slicen. Det explicita valet sparas lokalt i browsern;
utan ett sparat val används browserns standardsbaserade locale och därefter
svenska som fallback.

Valet skickas även som det typade fältet `response_locale` på varje agenttur.
Fältet valideras genom en exakt allowlist och blir auktoritativt för
`ANSWER`- och `CLARIFY`-texten. Modellen ska alltså inte gissa svarsspråk från
användartext, historik, evidens eller verktygsresultat. Det gör språkbyte
explicit och reproducerbart utan regexp, nyckelord eller språkheuristik.
Hosten kompletterar modellkontexten med locale-namn och en explicit
`response_language_instruction`, upprepar instruktionen på det genererade
textfältets JSON-schema och placerar den sist i den strukturerade kontexten.
Det behövdes i liveprov för att valt språk skulle vinna även över en prompt
och konversationshistorik som uttryckligen begär motsatt språk.

Tekniska identiteter som `weather.current`, eventtyper, ID:n, hashes,
modellnamn och rå JSON översätts aldrig. Etiketter, statusförklaringar,
tillgänglighetstext, placeholders, datum och tider följer däremot vald locale.
Fler språk läggs till som kataloger och tas därefter upp i samma explicita
locale-allowlist; agent- och säkerhetslogiken behöver inte förgrenas per
språk.

## Fem ytor

- **Arbetsbänk** visar en versionsmärkt konversation, pågående tur,
  verifierat slutsvar, typad aktivitet och eventuell evidens.
- **Kroppar** visar logiska robotar med controllers och perceptionskällor.
  EV3RSTORM är deklarerad men inte observerad när ingen fysisk probe har
  körts.
- **Händelser** visar den begränsade, append-only eventströmmen med
  korrelations-ID:n. Rå prompt, rå modelltext och fulla evidence-URL:er
  loggas inte.
- **Experiment** reserverar en read-only yta för reproducerbara episoder och
  befintliga experimentartefakter.
- **Inställningar** ändrar sessionsbundna agentbudgetar. Inställningarna
  versionskontrolleras och återställs när servern startas om.

## Dialog och kontext

Varje tur har ett explicit läge:

- `conversation` tillåter ett direkt lokalt svar och låter fortfarande
  modellen välja ett godkänt informationsverktyg om färsk data behövs.
- `research_required` kräver verifierad evidens innan modellen får svara.

Hosten klassificerar inte användartext med regexp, substringmatchning eller
nyckelord. Gemma får en typad `conversation_history` med tidigare synliga
user/assistant-meddelanden och returnerar ett strikt `CALL_TOOL`, `ANSWER`,
`CLARIFY` eller `ABORT`. Det enda registrerade externa verktyget i denna
version är `weather.current`.

Dialoghistoriken är ett konversationsminne, inte robotens framtida
världsmodell. Fysiska följdreferenser som “två gånger till” måste senare
bindas till en separat, explicit action-/state-kontext innan någon exekvering
kan bli aktuell.

## Säkerhetsgräns

Dashboardservern har ingen route för:

- motorer eller stoppkommandon
- RobotAPI eller MotionSupervisor
- SSH
- TTS
- uppladdning av events eller robotstatus

Att ett nodkort någon gång visar `online` kommer därför inte att innebära
att noden kan styras från dashboarden.

Webbgränsen använder dessutom:

- en slumpad 256-bitars sessionstoken per serverstart
- tokeniserad bootstrap-/asset-sökväg och tokenkrav på samtliga API-anrop,
  även läsningar
- exakt `Host`- och `Origin`-kontroll
- strikt JSON utan dubblettnycklar eller icke-finita tal
- fasta request- och responsegränser
- en explicit asset- och route-allowlist
- en strikt Content Security Policy utan externa assets
- socket-timeout och ett fast tak för samtidiga HTTP-handlers
- en begränsad jobbkö med en enda researchworker

Frontend anropar endast hostservern. Den kontaktar aldrig LM Studio,
Open-Meteo eller EV3 direkt.

## Komponentmodell

Registret skiljer på en logisk robot och dess noder:

```text
EV3RSTORM
├── EV3 Main · controller
├── Front Camera · framtida camera source
└── Microphone Array · framtida microphone source

Composite Lab Robot
├── Robot Inventor 51515 · framtida controller
├── BOOST Move Hub · framtida controller
├── Vision Node · framtida camera source
└── Audio Node · framtida microphone source
```

Alla poster är beskrivande och har `control_exposed: false`. Samma kontrakt
kan senare representera flera parallella controllers och perceptionskällor
utan att göra dashboarden till motorägare.
