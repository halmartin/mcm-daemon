# Maintainer: Andreas Boehler <andreas _AT_ aboehler.at>
pkgname=dns320l-daemon-hg
pkgver=1.0
pkgrel=1
pkgdesc="Simple System Daemon for D-Link DNS-320L"
arch=('arm')
url="http://www.aboehler.at/cms"
license=('GPL')
groups=()
depends=('iniparser')
makedepends=('mercurial')
provides=()
conflicts=()
replaces=()
backup=('etc/dns320l-daemon.ini')
options=()
install=
source=()
noextract=()
md5sums=() #generate with 'makepkg -g'

_hgroot=http://www.aboehler.at/hg/dns320l-daemon
_hgrepo=dns320l-daemon

build() {
  cd "$srcdir"
  msg "Connecting to Mercurial server...."

  if [[ -d "$_hgrepo" ]]; then
    cd "$_hgrepo"
    hg pull -u
    msg "The local files are updated."
  else
    hg clone "$_hgroot" "$_hgrepo"
  fi

  msg "Mercurial checkout done or server timeout"
  msg "Starting build..."

  rm -rf "$srcdir/$_hgrepo-build"
  cp -r "$srcdir/$_hgrepo" "$srcdir/$_hgrepo-build"
  cd "$srcdir/$_hgrepo-build"

  make
}

package() {
  cd "$srcdir/$_hgrepo-build"
  install -D -m 644 dns320l-daemon.service "$pkgdir/usr/lib/systemd/system/dns320l-daemon.service"
  install -D -m 755 dns320l-daemon "$pkgdir/usr/bin/dns320l-daemon"
  install -D -m 600 dns320l-daemon.ini "$pkgdir/etc/dns320l-daemon.ini"
}

# vim:set ts=2 sw=2 et:
