# Fonte do painel Mainsail

O arquivo `mainsail-v2.17.0-chameleon.patch` contem as mudancas aplicadas ao Mainsail oficial `v2.17.0`.

Para recompilar:

```bash
git clone --branch v2.17.0 https://github.com/mainsail-crew/mainsail.git
cd mainsail
git apply ../mainsail-v2.17.0-chameleon.patch
npm install
npm run build
```

O conteudo gerado em `dist/` deve ser compactado na raiz do ZIP e colocado em `installer/assets/mainsail-chameleon.zip`.
